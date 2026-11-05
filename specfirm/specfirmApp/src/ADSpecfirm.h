/*
 * areaDetector driver for JETI SPECFIRM spectrometer boards (the PE65 in
 * Thorlabs' CCT series compact spectrometers, the PE60_2, SDCM3/4) over an
 * asyn octet port: the board's CDC-ACM virtual COM port, its Ethernet port
 * or a fake on TCP.
 *
 * Every spectrum is one 1-D NDUInt16 NDArray [pixels] on address 0, the
 * detector's counts in stream order, stamped at the end of the exposure,
 * with the integration time, the averages, the board's microsecond clock
 * and the saturation level as NDAttributes. The wavelength of each pixel
 * index is the board's own fit polynomial (*PARA:FIT0..4), published as a
 * Float64 array parameter.
 *
 * Acquisition follows areaDetector: Acquire, ImageMode (Single / Multiple /
 * Continuous), NumImages, AcquireTime (the integration time, s),
 * NumExposures (the board's averages), AcquirePeriod. TriggerMode Internal
 * sends "*MEAS:LIGHT tint av 1" per frame (or "*MEAS:RAW", RawMode, whose
 * block carries the board clock); External puts the board in its trigger
 * mode 1, in which the TTL edge itself starts a measurement with the
 * board's stored integration time and averages (set from AcquireTime and
 * NumExposures when they differ) and the board sends BEL when it is done,
 * which the driver answers with "*FETCH:LIGHT 1".
 *
 * Nothing is written to the board at start: every setting is read from it
 * and the output records take their initial values from the driver. The
 * driver writes *PARA:TRIG, *PARA:TRSL, and in External mode *PARA:TINT and
 * *PARA:AVER, all RAM settings the board forgets at power-off; it never
 * sends *PARA:SAVE, *PARA:DEF, *RST, *BOOT, a *CALIB, a *MMEM, a sensor,
 * ADC or network parameter, or the shutter motor.
 *
 * Written from JETI's "Operating Instructions Firmware SPECFIRM_xxx",
 * document revision 170 (2024), and the Thorlabs build's answers recorded in
 * geecs-epics (services/undulator/spectrometers/ccttest1/README.md).
 */

#pragma once

#include <epicsEvent.h>
#include <epicsTime.h>
#include <asynOctetSyncIO.h>

#include "ADDriver.h"

#define P_ConnectedString          "SPECFIRM_CONNECTED"       /* asynInt32   r/o */
#define P_LastErrorString          "SPECFIRM_LAST_ERROR"      /* asynOctet   r/o */
#define P_PixelsString             "SPECFIRM_PIXELS"          /* asynInt32   r/o */
#define P_WavelengthString         "SPECFIRM_WAVELENGTH"      /* asynFloat64Array r/o: nm per pixel index */
#define P_Fit0String               "SPECFIRM_FIT0"            /* asynFloat64 r/o */
#define P_Fit1String               "SPECFIRM_FIT1"
#define P_Fit2String               "SPECFIRM_FIT2"
#define P_Fit3String               "SPECFIRM_FIT3"
#define P_Fit4String               "SPECFIRM_FIT4"
#define P_DirectionString          "SPECFIRM_DIRECTION"       /* asynInt32   r/o: 0 lowest pixel first, 1 highest first */
#define P_MinIntegrationString     "SPECFIRM_MIN_TINT"        /* asynFloat64 r/o: s */
#define P_LevelMaxString           "SPECFIRM_LEVEL_MAX"       /* asynInt32   r/o: the frame's highest count */
#define P_LevelPercentString       "SPECFIRM_LEVEL_PCT"       /* asynInt32   r/o: of full scale */
#define P_SaturatedString          "SPECFIRM_SATURATED"       /* asynInt32   r/o */
#define P_BoardTimeString          "SPECFIRM_BOARD_TIME"      /* asynFloat64 r/o: s on the board's clock */
#define P_RawModeString            "SPECFIRM_RAW_MODE"        /* asynInt32   r/w: 0 *MEAS:LIGHT, 1 *MEAS:RAW */
#define P_TriggerSlopeString       "SPECFIRM_TRIGGER_SLOPE"   /* asynInt32   r/w: 0 rising, 1 falling */
#define P_SensorString             "SPECFIRM_SENSOR"          /* asynOctet   r/o */
#define P_BoardSerialString        "SPECFIRM_BOARD_SERIAL"    /* asynOctet   r/o */
#define P_SpectrometerSerialString "SPECFIRM_SPEC_SERIAL"     /* asynOctet   r/o */
#define P_StoredIntegrationString  "SPECFIRM_TINT_STORED"     /* asynFloat64 r/o: the board's *PARA:TINT, s */
#define P_StoredAveragesString     "SPECFIRM_AVER_STORED"     /* asynInt32   r/o: the board's *PARA:AVER */
#define P_BoardTriggerString       "SPECFIRM_BOARD_TRIGGER"   /* asynInt32   r/o: the board's *PARA:TRIG */

class ADSpecfirm : public ADDriver {
public:
    ADSpecfirm(const char *portName, const char *linePort, int maxPixels,
               int maxBuffers, size_t maxMemory);

    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value) override;
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value) override;
    asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                size_t nElements, size_t *nIn) override;
    void report(FILE *fp, int details) override;

    void acquisitionTask();

protected:
    int P_Connected;
#define FIRST_SPECFIRM_PARAM P_Connected
    int P_LastError, P_Pixels, P_Wavelength, P_Fit0, P_Fit1, P_Fit2, P_Fit3,
        P_Fit4, P_Direction, P_MinIntegration, P_LevelMax, P_LevelPercent,
        P_Saturated, P_BoardTime, P_RawMode, P_TriggerSlope, P_Sensor,
        P_BoardSerial, P_SpectrometerSerial, P_StoredIntegration,
        P_StoredAverages, P_BoardTrigger;

private:
    enum ReplyByte { ACK = 0x06, BEL = 0x07, NAK = 0x15, ETX = 0x03 };

    /* the line: every call with the driver lock held unless said otherwise */
    bool send(const char *command);
    size_t readBytes(char *buf, size_t n, double timeout);
    bool readByte(unsigned char *b, double timeout);
    bool readByteSkippingBell(unsigned char *b, double timeout);
    bool readLine(char *buf, size_t size, double timeout);
    bool query(const char *command, char *reply, size_t size, double timeout = 1.0);
    bool set(const char *command);
    void drain();
    void deviceError(char *buf, size_t size);

    /* the board */
    bool connect();
    void disconnect(const char *why);
    bool applyTrigger();
    bool applyStoredTiming(double tintMs, int averages);
    bool readBoardTime(double *seconds);
    void readStatus();

    /* acquisition */
    bool frameInternal(double tintMs, int averages, bool raw);
    bool frameExternal();
    bool readBlock(size_t header, size_t *pixels, double timeout);
    bool waitForBell(double limit, bool *stopped);
    void publish(const epicsTimeStamp &stamp, size_t n, double boardTime,
                 double tintMs, int averages);
    void finishAcquisition();
    void fail(const char *what, const char *detail);
    void setStatus(ADStatus_t status, const char *message);

    char linePort_[64];
    asynUser *pasynUserLine_;
    int maxPixels_;
    int pixels_;
    bool connected_;
    bool exposing_;          /* the line is waiting on a measurement: no status reads */
    int boardTrigger_;       /* *PARA:TRIG as the board holds it */
    int boardSlope_;         /* *PARA:TRSL */
    double storedTintMs_;    /* *PARA:TINT */
    int storedAverages_;     /* *PARA:AVER */
    double minTintMs_;
    double fit_[5];
    epicsFloat64 *wavelength_;
    epicsUInt16 *frame_;
    char *block_;            /* 8 + 2 * maxPixels bytes */
    epicsUInt32 rawHeaderUs_; /* the raw block's clock, low 32 bits */
    unsigned strayBells_;     /* BELs skipped where text or a length word was expected */
    int uniqueId_;
    epicsEventId wake_;
};
