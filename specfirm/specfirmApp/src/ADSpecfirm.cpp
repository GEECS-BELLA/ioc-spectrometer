/* ADSpecfirm: JETI SPECFIRM spectrometer boards as an areaDetector driver.
 * See ADSpecfirm.h for what it does and what it writes. */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "ADSpecfirm.h"

static const char *driverName = "ADSpecfirm";
static const char *driverVersion = "1.0.0";

static const double maxTintMs = 65000.0;      /* the manual's *PARA:TINT range */
static const int maxAverages = 10000;          /* the manual's av range */
static const unsigned fullScale = 65535;       /* 16-bit ADC */
static const double exposureSlack = 2.0;       /* s beyond tint * av before a measurement is given up */
static const size_t rawHeader = 8;             /* *MEAS:RAW format 1: a 32-bit microsecond clock and 4 zero bytes */

static void acquisitionTaskC(void *drvPvt)
{
    static_cast<ADSpecfirm *>(drvPvt)->acquisitionTask();
}

ADSpecfirm::ADSpecfirm(const char *portName, const char *linePort, int maxPixels,
                       int maxBuffers, size_t maxMemory)
    : ADDriver(portName, 1, 0, maxBuffers, maxMemory,
               asynFloat64ArrayMask, asynFloat64ArrayMask,
               ASYN_CANBLOCK, 1, 0, 0),
      pasynUserLine_(NULL), maxPixels_(maxPixels > 0 ? maxPixels : 4096), pixels_(0),
      connected_(false), exposing_(false), boardTrigger_(0), boardSlope_(0),
      storedTintMs_(0.0), storedAverages_(1), minTintMs_(0.01), rawHeaderUs_(0), strayBells_(0),
      strayBlocks_(0), uniqueId_(0)
{
    strncpy(linePort_, linePort, sizeof linePort_ - 1);
    linePort_[sizeof linePort_ - 1] = '\0';
    for (int i = 0; i < 5; i++)
        fit_[i] = 0.0;
    wavelength_ = static_cast<epicsFloat64 *>(calloc(maxPixels_, sizeof(epicsFloat64)));
    frame_ = static_cast<epicsUInt16 *>(calloc(maxPixels_, sizeof(epicsUInt16)));
    block_ = static_cast<char *>(calloc(rawHeader + 2 * maxPixels_, 1));
    wake_ = epicsEventCreate(epicsEventEmpty);

    createParam(P_ConnectedString, asynParamInt32, &P_Connected);
    createParam(P_LastErrorString, asynParamOctet, &P_LastError);
    createParam(P_PixelsString, asynParamInt32, &P_Pixels);
    createParam(P_WavelengthString, asynParamFloat64Array, &P_Wavelength);
    createParam(P_Fit0String, asynParamFloat64, &P_Fit0);
    createParam(P_Fit1String, asynParamFloat64, &P_Fit1);
    createParam(P_Fit2String, asynParamFloat64, &P_Fit2);
    createParam(P_Fit3String, asynParamFloat64, &P_Fit3);
    createParam(P_Fit4String, asynParamFloat64, &P_Fit4);
    createParam(P_DirectionString, asynParamInt32, &P_Direction);
    createParam(P_MinIntegrationString, asynParamFloat64, &P_MinIntegration);
    createParam(P_LevelMaxString, asynParamInt32, &P_LevelMax);
    createParam(P_LevelPercentString, asynParamInt32, &P_LevelPercent);
    createParam(P_SaturatedString, asynParamInt32, &P_Saturated);
    createParam(P_BoardTimeString, asynParamFloat64, &P_BoardTime);
    createParam(P_RawModeString, asynParamInt32, &P_RawMode);
    createParam(P_TriggerSlopeString, asynParamInt32, &P_TriggerSlope);
    createParam(P_SensorString, asynParamOctet, &P_Sensor);
    createParam(P_BoardSerialString, asynParamOctet, &P_BoardSerial);
    createParam(P_SpectrometerSerialString, asynParamOctet, &P_SpectrometerSerial);
    createParam(P_StoredIntegrationString, asynParamFloat64, &P_StoredIntegration);
    createParam(P_StoredAveragesString, asynParamInt32, &P_StoredAverages);
    createParam(P_BoardTriggerString, asynParamInt32, &P_BoardTrigger);

    setStringParam(ADManufacturer, "JETI / Thorlabs");
    setStringParam(ADModel, "SPECFIRM board");
    setStringParam(ADSDKVersion, "SPECFIRM firmware commands, rev. 170");
    setStringParam(NDDriverVersion, driverVersion);
    setStringParam(ADStatusMessage, "not connected");
    setStringParam(P_LastError, "");
    setIntegerParam(P_Connected, 0);
    setIntegerParam(P_Pixels, 0);
    setIntegerParam(P_RawMode, 0);
    setIntegerParam(P_LevelMax, 0);
    setIntegerParam(P_LevelPercent, 0);
    setIntegerParam(P_Saturated, 0);
    setDoubleParam(P_BoardTime, 0.0);
    setDoubleParam(P_MinIntegration, minTintMs_ / 1000.0);
    setIntegerParam(ADMaxSizeX, maxPixels_);
    setIntegerParam(ADMaxSizeY, 1);
    setIntegerParam(ADSizeX, maxPixels_);
    setIntegerParam(ADSizeY, 1);
    setIntegerParam(NDArraySizeX, maxPixels_);
    setIntegerParam(NDArraySizeY, 1);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(NDNDimensions, 1);
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, ADStatusDisconnected);
    setIntegerParam(ADImageMode, ADImageContinuous);
    setIntegerParam(ADTriggerMode, ADTriggerInternal);
    setIntegerParam(ADNumImages, 1);
    setIntegerParam(ADNumExposures, 1);
    setDoubleParam(ADAcquireTime, 0.1);
    setDoubleParam(ADAcquirePeriod, 0.0);
    setDoubleParam(ADTemperatureActual, 0.0);

    asynStatus status = pasynOctetSyncIO->connect(linePort_, 0, &pasynUserLine_, NULL);
    if (status != asynSuccess) {
        printf("%s: cannot connect to asyn port %s\n", driverName, linePort_);
    } else {
        /* binary and text share the line: no terminators, the driver frames */
        pasynOctetSyncIO->setInputEos(pasynUserLine_, "", 0);
        pasynOctetSyncIO->setOutputEos(pasynUserLine_, "", 0);
        lock();
        connect();
        callParamCallbacks();
        unlock();
    }

    if (!epicsThreadCreate("ADSpecfirm", epicsThreadPriorityMedium,
                           epicsThreadGetStackSize(epicsThreadStackMedium),
                           acquisitionTaskC, this))
        printf("%s: epicsThreadCreate failed\n", driverName);
}

/* ------------------------------------------------------------------------ */
/* the line                                                                 */

/* What arrived before a command (a late block, a trigger edge's BEL):
 * discarded with short reads, a bounded number of them. Not asyn's port
 * flush: the IP port's keeps reading while bytes keep coming, and a board
 * in trigger mode 1 with a 10 Hz edge sends one BEL every 100 ms for ever
 * (the driver hung there on the fake, 2026-11-06). */
void ADSpecfirm::discardPending()
{
    char junk[4096];
    for (int i = 0; i < 16; i++)             /* 64 KB, at once when buffered */
        if (readBytes(junk, sizeof junk, 0.01) == 0)
            break;
}

bool ADSpecfirm::send(const char *command)
{
    char line[128];
    snprintf(line, sizeof line, "%s\r", command);
    size_t nwrite = strlen(line), nactual = 0;
    if (!pasynUserLine_)
        return false;
    discardPending();
    asynStatus status = pasynOctetSyncIO->write(pasynUserLine_, line, nwrite, 1.0, &nactual);
    setStringParam(ADStringToServer, command);
    if (status != asynSuccess || nactual != nwrite) {
        fail("write failed", command);
        return false;
    }
    return true;
}

/* Up to n bytes within timeout (per read call, so a stalled stream ends
 * early); returns how many arrived. */
size_t ADSpecfirm::readBytes(char *buf, size_t n, double timeout)
{
    size_t got = 0;
    while (got < n) {
        size_t nread = 0;
        int eom = 0;
        asynStatus status = pasynOctetSyncIO->read(pasynUserLine_, buf + got, n - got,
                                                   timeout, &nread, &eom);
        got += nread;
        if (status != asynSuccess && nread == 0)
            break;
    }
    return got;
}

bool ADSpecfirm::readByte(unsigned char *b, double timeout)
{
    return readBytes(reinterpret_cast<char *>(b), 1, timeout) == 1;
}

/* A byte where text or a length word is expected: a BEL there is the
 * board's only unsolicited byte (a trigger edge's "done" while the board is
 * in its trigger mode after a stop), skipped and counted. */
bool ADSpecfirm::readByteSkippingBell(unsigned char *b, double timeout)
{
    for (int skipped = 0; skipped < 100; skipped++) {
        if (!readByte(b, timeout))
            return false;
        if (*b != BEL)
            return true;
        strayBells_++;
    }
    return false;      /* a BEL stream faster than the reply: give up as a timeout would */
}

/* One CR-terminated answer, the CR dropped. */
bool ADSpecfirm::readLine(char *buf, size_t size, double timeout)
{
    size_t i = 0;
    for (;;) {
        unsigned char b;
        if (!readByteSkippingBell(&b, i == 0 ? timeout : 0.5)) {
            buf[i] = '\0';
            return false;
        }
        if (b == '\r' || b == ETX) {
            buf[i] = '\0';
            return true;
        }
        if (i + 1 < size)
            buf[i++] = static_cast<char>(b);
    }
}

void ADSpecfirm::deviceError(char *buf, size_t size)
{
    buf[0] = '\0';
    if (send("*STAT:TXTERR?"))
        readLine(buf, size, 1.0);
    char *s = buf;
    while (*s == ' ')
        s++;
    memmove(buf, s, strlen(s) + 1);
}

/* A query: the answer line, or false with the board's reason in LastError.
 * A board left in its trigger mode 3 by another client sends BEL and a
 * whole data block on every edge: a reply that starts with a byte that is
 * no text (the block's length word) is drained and the query sent again,
 * three times in all; a board that floods the line faster than that is
 * reported in LastError, and an acquisition start takes it out of mode 3. */
bool ADSpecfirm::query(const char *command, char *reply, size_t size, double timeout)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        reply[0] = '\0';
        if (!send(command))
            return false;
        unsigned char first;
        if (!readByteSkippingBell(&first, timeout)) {
            fail("no answer to", command);
            return false;
        }
        if (first == NAK) {
            char why[128];
            deviceError(why, sizeof why);
            fail(command, why);
            return false;
        }
        if (first == '\r' || first == ETX) {
            setStringParam(ADStringFromServer, "");
            return true;
        }
        if (first < 0x20 || first == ACK) {
            strayBlocks_++;
            drain();
            continue;
        }
        reply[0] = static_cast<char>(first);
        reply[1] = '\0';
        bool ok = readLine(reply + 1, size - 1, 0.5);
        /* a block's remnant can begin with a printable byte: the board's
         * text is ASCII plus the degree and micro signs (Latin-1) */
        bool text = true;
        for (const unsigned char *p = reinterpret_cast<const unsigned char *>(reply); *p; p++)
            if (*p < 0x20 || (*p >= 0x7F && *p != 0xB0 && *p != 0xB5)) {
                text = false;
                break;
            }
        if (!text) {
            strayBlocks_++;
            drain();
            continue;
        }
        setStringParam(ADStringFromServer, reply);
        return ok;
    }
    fail("no text answer to (the board floods the line: a trigger mode 3 with a fast trigger?)", command);
    return false;
}

/* A setting: ACK, or false with the reason. Bytes that are neither (the
 * remnants of a foreign mode 3's block flood, which stops once the setting
 * takes) are skipped and counted, within a bounded budget: every setting
 * the driver makes is verified by a query afterwards, so a 0x06 inside a
 * block's data cannot pass as an acknowledgement unnoticed. */
bool ADSpecfirm::set(const char *command)
{
    if (!send(command))
        return false;
    unsigned char b;
    double timeout = 1.0;
    for (size_t skipped = 0; skipped < 65536; skipped++) {
        if (!readByte(&b, timeout)) {
            fail(skipped ? "no acknowledgement in the stream after" : "no answer to", command);
            return false;
        }
        if (b == ACK)
            return true;
        if (b == NAK) {
            char why[128];
            deviceError(why, sizeof why);
            fail(command, why);
            return false;
        }
        if (b == BEL)
            strayBells_++;
        else if (skipped == 0)
            strayBlocks_++;
        timeout = 0.2;
    }
    fail("no acknowledgement within 64 KB after", command);
    drain();
    return false;
}

/* Whatever the board still sends (a late block, a stray BEL): read until
 * quiet, for at most about two seconds: a board in a foreign trigger mode
 * 3 with a fast trigger is never quiet, and the driver must not hang on
 * it (seen on the fake at 10 Hz). */
void ADSpecfirm::drain()
{
    char junk[4096];
    for (int i = 0; i < 32; i++)             /* 128 KB, at once when buffered */
        if (readBytes(junk, sizeof junk, 0.2) == 0)
            break;
}

/* ------------------------------------------------------------------------ */
/* the board                                                                */

static double leadingNumber(const char *s)
{
    return strtod(s, NULL);
}

/* Identity and every setting the driver shows: queries only. */
bool ADSpecfirm::connect()
{
    char reply[256];
    if (!query("*IDN?", reply, sizeof reply, 2.0))
        return false;
    setStringParam(ADModel, reply);
    if (query("*VERS?", reply, sizeof reply))
        setStringParam(ADFirmwareVersion, reply);
    if (query("*PARA:SERN?", reply, sizeof reply))
        setStringParam(P_BoardSerial, reply);
    if (query("*PARA:SPNUM?", reply, sizeof reply))
        setStringParam(P_SpectrometerSerial, reply);
    if (query("*PARA:DEVNUM?", reply, sizeof reply))
        setStringParam(ADSerialNumber, reply);
    if (query("*PARA:SENS?", reply, sizeof reply))
        setStringParam(P_Sensor, reply);
    if (!query("*PARA:PIXEL?", reply, sizeof reply))
        return false;
    int pixels = atoi(reply);
    if (pixels < 1 || pixels > maxPixels_) {
        char msg[96];
        snprintf(msg, sizeof msg, "%d pixels, this IOC was built for at most %d", pixels, maxPixels_);
        fail("pixel count", msg);
        return false;
    }
    pixels_ = pixels;
    setIntegerParam(P_Pixels, pixels_);
    setIntegerParam(ADMaxSizeX, pixels_);
    setIntegerParam(ADSizeX, pixels_);
    setIntegerParam(NDArraySizeX, pixels_);
    setIntegerParam(NDArraySize, pixels_ * static_cast<int>(sizeof(epicsUInt16)));
    if (query("*PARA:DIRECT?", reply, sizeof reply))
        setIntegerParam(P_Direction, atoi(reply));
    static const char *fitQueries[5] = {"*PARA:FIT0?", "*PARA:FIT1?", "*PARA:FIT2?", "*PARA:FIT3?", "*PARA:FIT4?"};
    int fitParams[5] = {P_Fit0, P_Fit1, P_Fit2, P_Fit3, P_Fit4};
    for (int i = 0; i < 5; i++) {
        if (query(fitQueries[i], reply, sizeof reply))
            fit_[i] = leadingNumber(reply);
        setDoubleParam(fitParams[i], fit_[i]);
    }
    for (int p = 0; p < pixels_; p++) {
        double x = p, v = 0.0, xn = 1.0;
        for (int i = 0; i < 5; i++) {
            v += fit_[i] * xn;
            xn *= x;
        }
        wavelength_[p] = v;
    }
    if (query("*CONF:MINTIN?", reply, sizeof reply)) {
        double v = leadingNumber(reply);
        if (v > 0.0)
            minTintMs_ = v;
    }
    setDoubleParam(P_MinIntegration, minTintMs_ / 1000.0);
    if (query("*PARA:TINT?", reply, sizeof reply)) {
        storedTintMs_ = leadingNumber(reply);
        setDoubleParam(P_StoredIntegration, storedTintMs_ / 1000.0);
        setDoubleParam(ADAcquireTime, storedTintMs_ / 1000.0);
    }
    if (query("*PARA:AVER?", reply, sizeof reply)) {
        storedAverages_ = atoi(reply);
        setIntegerParam(P_StoredAverages, storedAverages_);
        setIntegerParam(ADNumExposures, storedAverages_);
    }
    if (query("*PARA:TRIG?", reply, sizeof reply)) {
        boardTrigger_ = atoi(reply);
        setIntegerParam(P_BoardTrigger, boardTrigger_);
        /* the measure modes (1, 3) are External; 0 and the enquiry mode (2)
         * Internal. Nothing is written here: the setpoint takes this value
         * and a later write of the same value changes nothing. */
        setIntegerParam(ADTriggerMode, (boardTrigger_ == 1 || boardTrigger_ == 3)
                                           ? ADTriggerExternal : ADTriggerInternal);
    }
    if (query("*PARA:TRSL?", reply, sizeof reply)) {
        boardSlope_ = atoi(reply);
        setIntegerParam(P_TriggerSlope, boardSlope_);
    }
    connected_ = true;
    setIntegerParam(P_Connected, 1);
    setStringParam(P_LastError, "");
    readStatus();
    setStatus(ADStatusIdle, "connected");
    doCallbacksFloat64Array(wavelength_, pixels_, P_Wavelength, 0);
    return true;
}

void ADSpecfirm::disconnect(const char *why)
{
    connected_ = false;
    setIntegerParam(P_Connected, 0);
    setIntegerParam(ADAcquire, 0);
    setStatus(ADStatusDisconnected, why);
}

/* Temperature, the shutter motor and the board's error word: idle only. */
void ADSpecfirm::readStatus()
{
    char reply[128];
    if (!connected_ || exposing_)
        return;
    if (query("*MEAS:TEMPE", reply, sizeof reply, 3.0))
        setDoubleParam(ADTemperatureActual, leadingNumber(reply));
    if (query("*CONTR:MOTOR?", reply, sizeof reply))
        setIntegerParam(ADShutterStatus, atoi(reply));
    if (query("*STAT:TXTERR?", reply, sizeof reply)) {
        char *s = reply;
        while (*s == ' ')
            s++;
        if (atoi(s) != 0)
            setStringParam(P_LastError, s);
    }
}

bool ADSpecfirm::readBoardTime(double *seconds)
{
    char reply[64];
    if (!query("*CONTR:TIMESTAMP?", reply, sizeof reply))
        return false;
    *seconds = leadingNumber(reply) / 1e6;
    return true;
}

/* The board's trigger mode and slope follow TriggerMode and TriggerSlope:
 * written only when they differ from what the board holds (RAM). */
bool ADSpecfirm::applyTrigger()
{
    int mode = ADTriggerInternal, slope = 0;
    getIntegerParam(ADTriggerMode, &mode);
    getIntegerParam(P_TriggerSlope, &slope);
    int want = mode == ADTriggerExternal ? 1 : 0;
    char cmd[32], reply[64];
    if (want != boardTrigger_) {
        snprintf(cmd, sizeof cmd, "*PARA:TRIG %d", want);
        if (!set(cmd))
            return false;
        if (query("*PARA:TRIG?", reply, sizeof reply))
            boardTrigger_ = atoi(reply);
        setIntegerParam(P_BoardTrigger, boardTrigger_);
        if (boardTrigger_ != want) {
            fail("trigger mode not taken", reply);
            return false;
        }
    }
    if (slope != boardSlope_) {
        snprintf(cmd, sizeof cmd, "*PARA:TRSL %d", slope ? 1 : 0);
        if (!set(cmd))
            return false;
        if (query("*PARA:TRSL?", reply, sizeof reply))
            boardSlope_ = atoi(reply);
        setIntegerParam(P_TriggerSlope, boardSlope_);
    }
    return true;
}

/* External mode measures with the board's stored integration time and
 * averages: set them (RAM) when the setpoints differ. */
bool ADSpecfirm::applyStoredTiming(double tintMs, int averages)
{
    char cmd[48], reply[64];
    if (fabs(tintMs - storedTintMs_) > 0.0005) {
        snprintf(cmd, sizeof cmd, "*PARA:TINT %.3f", tintMs);
        if (!set(cmd))
            return false;
        if (query("*PARA:TINT?", reply, sizeof reply))
            storedTintMs_ = leadingNumber(reply);
        setDoubleParam(P_StoredIntegration, storedTintMs_ / 1000.0);
    }
    if (averages != storedAverages_) {
        snprintf(cmd, sizeof cmd, "*PARA:AVER %d", averages);
        if (!set(cmd))
            return false;
        if (query("*PARA:AVER?", reply, sizeof reply))
            storedAverages_ = atoi(reply);
        setIntegerParam(P_StoredAverages, storedAverages_);
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* acquisition                                                              */

void ADSpecfirm::acquisitionTask()
{
    lock();
    for (;;) {
        if (!connected_) {
            unlock();
            epicsEventWaitWithTimeout(wake_, 5.0);
            lock();
            if (pasynUserLine_ && !connected_)
                connect();
            callParamCallbacks();
            continue;
        }
        int acquire = 0;
        getIntegerParam(ADAcquire, &acquire);
        if (!acquire) {
            int state = ADStatusIdle;
            getIntegerParam(ADStatus, &state);
            if (state == ADStatusAcquire || state == ADStatusReadout || state == ADStatusWaiting)
                setStatus(ADStatusIdle, "stopped");
            callParamCallbacks();
            unlock();
            epicsEventWait(wake_);
            lock();
            continue;
        }

        double tint = 0.0, period = 0.0;
        int averages = 1, mode = ADTriggerInternal, raw = 0;
        getDoubleParam(ADAcquireTime, &tint);
        getDoubleParam(ADAcquirePeriod, &period);
        getIntegerParam(ADNumExposures, &averages);
        getIntegerParam(ADTriggerMode, &mode);
        getIntegerParam(P_RawMode, &raw);
        double tintMs = tint * 1000.0;
        if (tintMs < minTintMs_)
            tintMs = minTintMs_;
        if (tintMs > maxTintMs)
            tintMs = maxTintMs;
        if (averages < 1)
            averages = 1;
        if (averages > maxAverages)
            averages = maxAverages;
        setDoubleParam(ADAcquireTime, tintMs / 1000.0);
        setIntegerParam(ADNumExposures, averages);

        epicsTimeStamp start;
        epicsTimeGetCurrent(&start);
        bool ok;
        if (!applyTrigger()) {
            ok = false;
        } else if (mode == ADTriggerExternal) {
            ok = applyStoredTiming(tintMs, averages) && frameExternal();
        } else {
            ok = frameInternal(tintMs, averages, raw != 0);
        }
        if (!connected_) {
            callParamCallbacks();
            continue;
        }
        if (!ok) {
            setIntegerParam(ADAcquire, 0);
            setIntegerParam(ADStatus, ADStatusError);
            callParamCallbacks();
            continue;
        }

        getIntegerParam(ADAcquire, &acquire);
        if (acquire && period > 0.0) {
            epicsTimeStamp now;
            epicsTimeGetCurrent(&now);
            double remaining = period - epicsTimeDiffInSeconds(&now, &start);
            if (remaining > 0.0) {
                setStatus(ADStatusWaiting, "waiting for the period");
                callParamCallbacks();
                unlock();
                epicsEventWaitWithTimeout(wake_, remaining);
                lock();
            }
        }
        callParamCallbacks();
    }
}

/* BEL from the board, in slices so a stop is seen (the lock released while
 * reading). Returns false after `limit` seconds; limit < 0 waits forever. */
bool ADSpecfirm::waitForBell(double limit, bool *stopped)
{
    epicsTimeStamp t0;
    epicsTimeGetCurrent(&t0);
    *stopped = false;
    exposing_ = true;
    for (;;) {
        unsigned char b = 0;
        unlock();
        bool got = readByte(&b, 0.5);
        lock();
        if (got) {
            exposing_ = false;
            if (b == BEL)
                return true;
            if (b == NAK) {
                char why[128];
                deviceError(why, sizeof why);
                fail("the measurement was refused", why);
            } else {
                fail("unexpected byte while waiting for the measurement", "");
                drain();
            }
            return false;
        }
        int acquire = 0;
        getIntegerParam(ADAcquire, &acquire);
        if (!acquire && !*stopped) {
            *stopped = true;
            if (limit < 0.0) {
                exposing_ = false;
                return false;         /* a trigger wait ends at once */
            }
        }
        epicsTimeStamp now;
        epicsTimeGetCurrent(&now);
        double elapsed = epicsTimeDiffInSeconds(&now, &t0);
        if (limit >= 0.0 && elapsed > limit) {
            exposing_ = false;
            fail("no measurement within the exposure time", "");
            drain();
            return false;
        }
        if (limit < 0.0) {
            char msg[64];
            snprintf(msg, sizeof msg, "waiting for the trigger (%.0f s)", elapsed);
            setStringParam(ADStatusMessage, msg);
            callParamCallbacks();
        }
    }
}

/* The block after BEL (or after *FETCH): a 16-bit byte count, then
 * `header` bytes, then little-endian 16-bit pixels into frame_. */
bool ADSpecfirm::readBlock(size_t header, size_t *pixels, double timeout)
{
    unsigned char len[2];
    if (!readByteSkippingBell(&len[0], timeout) || !readByte(&len[1], timeout)) {
        fail("no block after the measurement", "");
        return false;
    }
    size_t nbytes = len[0] | (static_cast<size_t>(len[1]) << 8);
    size_t limit = header + 2 * static_cast<size_t>(maxPixels_);
    if (nbytes < header || ((nbytes - header) & 1) || nbytes > limit) {
        char msg[64];
        snprintf(msg, sizeof msg, "%zu bytes", nbytes);
        fail("block size", msg);
        drain();
        return false;
    }
    size_t got = readBytes(block_, nbytes, timeout);
    if (got != nbytes) {
        char msg[64];
        snprintf(msg, sizeof msg, "%zu of %zu bytes", got, nbytes);
        fail("short block", msg);
        drain();
        return false;
    }
    size_t n = (nbytes - header) / 2;
    const unsigned char *p = reinterpret_cast<const unsigned char *>(block_) + header;
    for (size_t i = 0; i < n; i++)
        frame_[i] = static_cast<epicsUInt16>(p[2 * i] | (p[2 * i + 1] << 8));
    *pixels = n;
    return true;
}

/* Internal: "*MEAS:LIGHT tint av 1" (RAW with the board clock in its header). */
bool ADSpecfirm::frameInternal(double tintMs, int averages, bool raw)
{
    char cmd[64];
    snprintf(cmd, sizeof cmd, "*MEAS:%s %.3f %d 1", raw ? "RAW" : "LIGHT", tintMs, averages);
    if (!send(cmd))
        return false;
    unsigned char first;
    if (!readByteSkippingBell(&first, 2.0)) {
        fail("no answer to", cmd);
        disconnect("the board does not answer");
        return false;
    }
    if (first == NAK) {
        char why[128];
        deviceError(why, sizeof why);
        fail(cmd, why);
        return false;
    }
    if (first != ACK) {
        fail("unexpected byte after", cmd);
        drain();
        return false;
    }
    setStatus(ADStatusAcquire, "exposing");
    callParamCallbacks();
    bool stopped = false;
    if (!waitForBell(tintMs * averages / 1000.0 + exposureSlack, &stopped))
        return false;
    epicsTimeStamp stamp;
    epicsTimeGetCurrent(&stamp);
    setStatus(ADStatusReadout, "reading out");
    size_t n = 0;
    if (!readBlock(raw ? rawHeader : 0, &n, 2.0))
        return false;
    /* The board's clock after the readout, from its query (the microsecond
     * count as text, unbounded). The raw block's header carries only the
     * low 32 bits of that count, which wrap every 71.6 min (seen on the
     * unit 2026-11-05): kept as RawHeaderTime, not used for BoardTime. */
    double boardTime = NAN;
    readBoardTime(&boardTime);
    rawHeaderUs_ = 0;
    if (raw) {
        const unsigned char *h = reinterpret_cast<const unsigned char *>(block_);
        rawHeaderUs_ = h[0] | (h[1] << 8) | (h[2] << 16) | (static_cast<epicsUInt32>(h[3]) << 24);
    }
    if (stopped) {
        setStatus(ADStatusIdle, "stopped: the last exposure discarded");
        return true;
    }
    publish(stamp, n, boardTime, tintMs, averages);
    return true;
}

/* External: the trigger starts the measurement; BEL says it is done. */
bool ADSpecfirm::frameExternal()
{
    setStatus(ADStatusAcquire, "waiting for the trigger");
    callParamCallbacks();
    bool stopped = false;
    if (!waitForBell(-1.0, &stopped)) {
        if (stopped)
            setStatus(ADStatusIdle, "stopped");
        return stopped;
    }
    epicsTimeStamp stamp;
    epicsTimeGetCurrent(&stamp);
    setStatus(ADStatusReadout, "reading out");
    double boardTime = NAN;
    readBoardTime(&boardTime);
    if (!send("*FETCH:LIGHT 1"))
        return false;
    size_t n = 0;
    if (!readBlock(0, &n, 2.0))
        return false;
    publish(stamp, n, boardTime, storedTintMs_, storedAverages_);
    return true;
}

void ADSpecfirm::publish(const epicsTimeStamp &stamp, size_t n, double boardTime,
                         double tintMs, int averages)
{
    unsigned maxCount = 0;
    for (size_t i = 0; i < n; i++)
        if (frame_[i] > maxCount)
            maxCount = frame_[i];
    int percent = static_cast<int>(maxCount * 100ull / (fullScale + 1ull));
    int saturated = maxCount >= fullScale;
    setIntegerParam(P_LevelMax, static_cast<int>(maxCount));
    setIntegerParam(P_LevelPercent, percent);
    setIntegerParam(P_Saturated, saturated);
    if (!std::isnan(boardTime))
        setDoubleParam(P_BoardTime, boardTime);

    int counter = 0, imagesCounter = 0, callbacks = 1;
    getIntegerParam(NDArrayCounter, &counter);
    setIntegerParam(NDArrayCounter, ++counter);
    getIntegerParam(ADNumImagesCounter, &imagesCounter);
    setIntegerParam(ADNumImagesCounter, ++imagesCounter);
    getIntegerParam(NDArrayCallbacks, &callbacks);
    setTimeStamp(&stamp);
    setIntegerParam(NDArraySizeX, static_cast<int>(n));
    setIntegerParam(NDArraySize, static_cast<int>(n * sizeof(epicsUInt16)));
    doCallbacksFloat64Array(wavelength_, pixels_, P_Wavelength, 0);

    if (callbacks) {
        size_t dims[1] = {n};
        NDArray *array = pNDArrayPool->alloc(1, dims, NDUInt16, 0, NULL);
        if (!array) {
            setStringParam(P_LastError, "NDArray pool exhausted: spectrum dropped");
        } else {
            memcpy(array->pData, frame_, n * sizeof(epicsUInt16));
            array->uniqueId = uniqueId_++;
            array->timeStamp = stamp.secPastEpoch + stamp.nsec / 1.e9;
            array->epicsTS = stamp;
            getAttributes(array->pAttributeList);
            int avg = averages, maxC = static_cast<int>(maxCount), pix = static_cast<int>(n);
            array->pAttributeList->add("IntegrationTime", "Integration time (ms)", NDAttrFloat64, &tintMs);
            array->pAttributeList->add("Averages", "Scans averaged by the board", NDAttrInt32, &avg);
            array->pAttributeList->add("BoardTime", "The board's clock after the readout (s)", NDAttrFloat64, &boardTime);
            if (rawHeaderUs_) {
                double rawHeaderS = rawHeaderUs_ / 1e6;
                array->pAttributeList->add("RawHeaderTime", "The raw block's clock, low 32 bits (s, wraps)", NDAttrFloat64, &rawHeaderS);
            }
            array->pAttributeList->add("LevelMax", "Highest count", NDAttrInt32, &maxC);
            array->pAttributeList->add("Saturated", "A pixel at full scale", NDAttrInt32, &saturated);
            array->pAttributeList->add("Pixels", "Pixels in the spectrum", NDAttrInt32, &pix);
            if (pArrays[0])
                pArrays[0]->release();
            pArrays[0] = array;
            doCallbacksGenericPointer(array, NDArrayData, 0);
        }
    }

    int imageMode = ADImageContinuous, numImages = 1, acquire = 1;
    getIntegerParam(ADImageMode, &imageMode);
    getIntegerParam(ADNumImages, &numImages);
    getIntegerParam(ADAcquire, &acquire);
    if (imageMode == ADImageSingle ||
        (imageMode == ADImageMultiple && imagesCounter >= numImages))
        finishAcquisition();
    else if (!acquire)
        setStatus(ADStatusIdle, "stopped");
    else
        setStatus(ADStatusAcquire, "acquiring");
}

void ADSpecfirm::finishAcquisition()
{
    setIntegerParam(ADAcquire, 0);
    setStatus(ADStatusIdle, "acquisition complete");
}

void ADSpecfirm::fail(const char *what, const char *detail)
{
    char msg[256];
    if (detail && *detail)
        snprintf(msg, sizeof msg, "%s: %s", what, detail);
    else
        snprintf(msg, sizeof msg, "%s", what);
    setStringParam(P_LastError, msg);
    setStringParam(ADStatusMessage, msg);
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s %s: %s\n", driverName, portName, msg);
}

void ADSpecfirm::setStatus(ADStatus_t status, const char *message)
{
    setIntegerParam(ADStatus, status);
    setStringParam(ADStatusMessage, message);
}

/* ------------------------------------------------------------------------ */
/* record writes                                                            */

asynStatus ADSpecfirm::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    if (function == ADAcquire) {
        int was = 0;
        getIntegerParam(ADAcquire, &was);
        setIntegerParam(ADAcquire, value);
        if (value && !was) {
            setIntegerParam(ADNumImagesCounter, 0);
            if (!connected_) {
                setIntegerParam(ADAcquire, 0);
                setStatus(ADStatusDisconnected, "not connected");
            }
        }
        epicsEventSignal(wake_);
    } else if (function == ADTriggerMode || function == P_TriggerSlope) {
        /* written to the board only when the value changes: the records'
         * PINI writes at start repeat the values read from the board */
        int current = 0;
        getIntegerParam(function, &current);
        setIntegerParam(function, value);
        if (value != current && connected_ && !exposing_)
            applyTrigger();
    } else if (function == ADReadStatus) {
        readStatus();
    } else if (function == NDDataType) {
        setIntegerParam(NDDataType, NDUInt16);
    } else if (function == ADShutterControl) {
        /* the shutter motor is not driven: what 1 and 0 do is not yet known */
        setIntegerParam(function, value);
    } else if (function < FIRST_SPECFIRM_PARAM) {
        status = ADDriver::writeInt32(pasynUser, value);
    } else {
        setIntegerParam(function, value);
    }
    callParamCallbacks();
    return status;
}

asynStatus ADSpecfirm::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    if (function == ADAcquireTime) {
        double ms = value * 1000.0;
        if (ms < minTintMs_)
            ms = minTintMs_;
        if (ms > maxTintMs)
            ms = maxTintMs;
        setDoubleParam(ADAcquireTime, ms / 1000.0);
    } else {
        setDoubleParam(function, value);
    }
    callParamCallbacks();
    return asynSuccess;
}

/* The wavelength axis on demand (the record's PINI, a caget-triggered
 * processing); I/O Intr callbacks carry it with every frame as well. */
asynStatus ADSpecfirm::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                        size_t nElements, size_t *nIn)
{
    if (pasynUser->reason != P_Wavelength)
        return ADDriver::readFloat64Array(pasynUser, value, nElements, nIn);
    size_t n = static_cast<size_t>(pixels_);
    if (n > nElements)
        n = nElements;
    memcpy(value, wavelength_, n * sizeof(epicsFloat64));
    *nIn = n;
    return asynSuccess;
}

void ADSpecfirm::report(FILE *fp, int details)
{
    char model[64] = "", version[64] = "";
    getStringParam(ADModel, sizeof model, model);
    getStringParam(ADFirmwareVersion, sizeof version, version);
    fprintf(fp, "ADSpecfirm %s on %s: %s (%s), %d pixels, trigger %d, stored %.3f ms x %d, %u stray BEL, %u stray blocks\n",
            portName, linePort_, connected_ ? model : "not connected", version, pixels_,
            boardTrigger_, storedTintMs_, storedAverages_, strayBells_, strayBlocks_);
    ADDriver::report(fp, details);
}

/* ------------------------------------------------------------------------ */
/* iocsh                                                                    */

extern "C" {

/** ADSpecfirmConfig(portName, linePort, maxPixels, maxBuffers, maxMemory):
 *  linePort is the asyn octet port of the board's line (a tty or a TCP
 *  socket); maxPixels sizes the buffers and the wavelength record;
 *  maxBuffers / maxMemory bound the NDArray pool (0 = unlimited). */
int ADSpecfirmConfig(const char *portName, const char *linePort, int maxPixels,
                     int maxBuffers, int maxMemory)
{
    new ADSpecfirm(portName, linePort, maxPixels, maxBuffers < 0 ? 0 : maxBuffers,
                   maxMemory < 0 ? 0 : maxMemory);
    return asynSuccess;
}

static const iocshArg configArg0 = {"portName", iocshArgString};
static const iocshArg configArg1 = {"linePort", iocshArgString};
static const iocshArg configArg2 = {"maxPixels", iocshArgInt};
static const iocshArg configArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg configArg4 = {"maxMemory", iocshArgInt};
static const iocshArg *const configArgs[] = {&configArg0, &configArg1, &configArg2,
                                             &configArg3, &configArg4};
static const iocshFuncDef configFuncDef = {"ADSpecfirmConfig", 5, configArgs};

static void configCallFunc(const iocshArgBuf *args)
{
    ADSpecfirmConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void ADSpecfirmRegister(void)
{
    iocshRegister(&configFuncDef, configCallFunc);
}

epicsExportRegistrar(ADSpecfirmRegister);
}
