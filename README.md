# Generic IOC ioc-spectrometer

Compact spectrometers as areaDetector drivers, for GEECS (geecs-epics
decision 14: a spectrum is an NDArray per capture, so the plugin chain,
HDF5 and PVA apply; decision 11: one image per family). Built on
epics-containers' `ioc-adaravis` developer image R2-3ec3, as ioc-picoscope
is: EPICS base, asyn, autosave, iocStats, calc, busy and ADCore (HDF5, PVA,
the plugins) already built, the same stack the camera and scope IOCs run,
plus:

| Module | Version | Source | Reuse (decision 10) |
|---|---|---|---|
| specfirm (`ADSpecfirm`) | this repository | ours: written from JETI's "Operating Instructions Firmware SPECFIRM_xxx", document revision 170 (2024), and the Thorlabs build's recorded answers (geecs-epics `services/undulator/spectrometers/ccttest1/`) | written (no EPICS driver for any JETI board existed, search of 2026-11-05 in the catalogue row) |

No vendor code: the board's firmware commands are documented, so CI
publishes this image. First device: a Thorlabs CCT compact spectrometer, a
JETI PE65 board (geecs-epics `services/undulator/spectrometers/ccttest1`).

## The driver

`ADSpecfirm` (`specfirm/specfirmApp/src/`) is an `ADDriver` over an asyn
octet port, the board's line: its CDC-ACM virtual COM port
(`asyn.AsynSerial`, any baud: the rate is symbolic, the transfer is USB high
speed), its Ethernet port or a fake (`asyn.AsynIP`). The driver sets the
port's terminators to none and frames the protocol itself: text replies end
in CR, a setting answers ACK (0x06) or NAK (0x15), a measurement answers ACK,
then BEL (0x07) when the exposure is over, then a block of a 16-bit byte
count and little-endian 16-bit counts.

- **Every spectrum** is one 1-D `NDUInt16` NDArray [pixels] on address 0,
  the counts in the board's stream order, stamped at the end of the
  exposure; attributes `IntegrationTime` (ms), `Averages`, `BoardTime` (s on
  the board's clock, from `*CONTR:TIMESTAMP?` after the readout),
  `RawHeaderTime` (Raw mode: the raw block's own clock, the low 32 bits of
  the same count, which wrap every 71.6 min), `LevelMax`, `Saturated`,
  `Pixels`.
- **Wavelength**: the board's fit polynomial `*PARA:FIT0..4` evaluated per
  pixel index, `Wavelength_RBV` (nm). Which end of the stream is pixel 0
  when the board's `DIRECTion` is 1 (highest pixel first) is a question for
  a line source on the bench; the driver does not reorder.
- **Acquisition** is areaDetector's: `Acquire`, `ImageMode` (Single /
  Multiple / Continuous), `NumImages`, `AcquireTime` (the integration time,
  s, clamped to the board's `*CONF:MINTIN?` .. 65 s), `NumExposures` (the
  board's averages, 1..10000), `AcquirePeriod`.
- **TriggerMode Internal**: each frame is `*MEAS:LIGHT tint av 1`
  (`RawMode` Raw: `*MEAS:RAW`, whose block starts with an 8-byte header),
  the BEL awaited for tint × av + 2 s, then `*CONTR:TIMESTAMP?` for
  `BoardTime`.
- **TriggerMode External**: the board's trigger mode 1, in which the TTL
  edge on its Trigger In starts a measurement with the board's **stored**
  integration time and averages (`*PARA:TINT`, `*PARA:AVER`, shown as
  `StoredIntegration_RBV` and `StoredAverages_RBV`) and the board sends BEL
  when it is done; the driver then sends `*FETCH:LIGHT 1`. When
  `AcquireTime` or `NumExposures` differ from the stored values at the start
  of an acquisition, the driver sets them (RAM). A stop while waiting for a
  trigger ends at once; the board stays in its trigger mode, so a later
  edge's BEL can arrive at any time: the driver skips a BEL wherever text or
  a length word is expected (counted in `report`; a hundred in a row ends
  the read as a timeout would), and `send` discards what arrived before a
  command with a few short reads, never asyn's port flush, whose IP-port
  form keeps reading while bytes keep coming (a 10 Hz edge in mode 1 hung
  the driver there on the fake). A board left in trigger mode 3 by another
  client sends BEL and a whole block on every edge: a text reply that begins
  with a byte that is no text, or that holds one (the board's text is ASCII
  plus the degree and micro signs), is drained (a bounded read: a fast
  trigger in mode 3 is never quiet) and the query sent again, three times
  in all, then `LastError_RBV` says the line is flooded (counted too);
  the driver itself never selects mode 3 (External is mode 1), and an
  acquisition start takes a board out of it: the setting's acknowledgement
  is found past the flood's remnants (every setting is verified by a query
  afterwards), and the discard and drain budgets cover a backlog of blocks.
  While such a flood lasts a query's value can still be wrong once (a
  remnant whose bytes are all printable: bright counts), and a status poll
  can hold the driver for tens of seconds in the worst case; the next poll
  or the next Acquire corrects it (geecs-epics ccttest1,
  `records/fake-review-2026-11-06.txt`). On an idle line the discard before
  a command waits for nothing, so the frame rate is the board's.
- **Status** every 10 s while idle (ADBase's `ReadStatus`, given `SCAN 10
  second` by the template):
  `*MEAS:TEMPE` into `TemperatureActual`, `*CONTR:MOTOR?` into
  `ShutterStatus_RBV` (the shutter motor's state as the board reports it;
  the driver does not drive it, `ShutterControl` is accepted and ignored
  until the bench says what 1 and 0 do), `*STAT:TXTERR?` into `LastError_RBV`
  when non-zero. Never during an exposure.
- **Level**: `LevelMax_RBV`, `LevelPercent_RBV` and `Saturated_RBV`
  (MAJOR) from each frame.

### What the driver writes, and never writes

Nothing at start: it reads `*IDN?`, `*VERS?`, the serial numbers, the sensor,
the pixel count, the direction, the fit, `*CONF:MINTIN?`, `*PARA:TINT?`,
`*PARA:AVER?`, `*PARA:TRIG?` and `*PARA:TRSL?`, and the output records take
their initial values from it (asyn's init read), so the PINI writes of
ADBase's records repeat the values just read and the driver writes the
trigger mode and slope only when a value changes. A board found in trigger
mode 1 or 3 reads as External, 0 or 2 as Internal. It writes, on the
operator's command, `*PARA:TRIG` and `*PARA:TRSL` (when `TriggerMode` or
`TriggerSlope` change, and at an acquisition start when the board holds a
mode other than the setpoint's: 1 for External, 0 for Internal) and, in
External mode, `*PARA:TINT` and `*PARA:AVER` (when the setpoints differ
from the stored values): RAM settings the board forgets at power-off. It never sends `*PARA:SAVE`,
`*PARA:DEF`, `*PARA:BACKUP`/`RESTORE`, `*RST`, `*BOOT`, `*USBRST`, `*SLEEP`,
any `*CALIB` or `*MMEM` command, the shutter motor, the LED, or a sensor,
ADC, fit or network parameter.

### Behaviour recorded on the Thorlabs build (2026-11-05)

`SPECFIRM_PE65_THORLABS 1.2.0`: BEL arrives tint × av + about 5 ms after
ACK and the 4 100-byte block within 1 ms; the first light measurement after
power-up, and again the first after an idle period of hours, carried 65535
in the last word of its stream (`Saturated_RBV` flags it; later frames are
clean); a measurement's arguments do not change the stored
`*PARA:TINT`/`AVER`; an unknown command NAKs with the error word still 0;
the raw block's header clock is the low 32 bits of `*CONTR:TIMESTAMP?`'s
count; the driver sustains about 93 spectra/s at 2.44 ms on the unit; the
board takes `*PARA:TRIG 1`/`0`, `*PARA:TRSL`, `*PARA:TINT` and `*PARA:AVER`
with ACK and reads them back, runs a software measurement at once even in
trigger mode, and its shutter motor takes `*CONTR:MOTOR 1`/`0` (not driven by
the driver; `*MEAS:DARK` does not move it). Details and the transcripts:
geecs-epics
`services/undulator/spectrometers/ccttest1/README.md`.

## The entity

`specfirm.ADSpecfirm` (`ibek-support-geecs/specfirm/specfirm.ibek.support.yaml`):
`ADSpecfirmConfig(PORT, LINE, MAX_PIXELS, MAX_BUFFERS, MAX_MEMORY)` and the
two templates, `ADSpecfirm.template` (the driver's records under `P R`,
ADBase included) and `ADSpecfirm_spectrometer.template` (the vendor-neutral
names aliased at `P`: `Acquire`, `ImageMode`, `NumImages`, `TriggerMode`,
`TriggerSlope`, `IntegrationTime`, `Averages`, `AcquirePeriod`, `RawMode`,
`Pixels_RBV`, `Wavelength_RBV`, `LevelMax_RBV`, `LevelPercent_RBV`,
`Saturated_RBV`, `BoardTime_RBV`, `Connected_RBV`, `DetectorState_RBV`,
`StatusMessage_RBV`, `LastError_RBV`, `ArrayCounter_RBV`,
`TemperatureActual`, the identity and the fit). `LINE` names the line's asyn
port, configured without asyn's EOS interpose: `noProcessEos=1` on
`drvAsynSerialPortConfigure` or `drvAsynIPPortConfigure` in a startup
command (geecs-epics' `include/serial-line` scripts take `NOEOS=1`), or
ibek's `asyn.AsynIP` entity, which passes `noProcessEos`; not ibek's
`asyn.AsynSerial`, which configures the port with the interpose and then
emits an `asynSetOption` for `noProcessEos` that the serial port rejects.
The driver frames its own bytes, and the interpose discards the bytes of a
read that `drvAsynSerialPort` reports as a timeout, which it does for a byte
arriving in the last ~0.1 s of a timed read: with the interpose the board's
BEL was lost on every frame of the exposure lengths whose BEL lands in the
window, about a fifth of those over 0.5 s (ccttest1, 2026-11-06). `MAX_PIXELS` (default 4096) sizes the buffers and
`Wavelength_RBV`; a board with more refuses to connect.

An instance: geecs-epics `services/undulator/spectrometers/ccttest1/config/ioc.yaml`,
with the chain (`NDStdArrays`, `NDPvxsPlugin`, `NDFileHDF5`, `NDProcess`
for a background, `NDStats`).

## Build

CI publishes `ghcr.io/geecs-bella/ioc-spectrometer-runtime:<tag>` and
`-developer` on a git tag (CalVer `YYYY.M.N`; a pre-release tag such as
`2026.11.1b1` for bench work from a branch). Locally:

    DOCKER_DEFAULT_PLATFORM=linux/amd64 TARGET=runtime TAG=ioc-spectrometer:local ./build

(the variable is required on Apple silicon). `tests/run-tests.sh <tag>` starts
the image with `tests/config` (no hardware).

## Licence

The driver and this repository: `LICENSE` (the GEECS licence, as the other
GEECS-BELLA images); the template's Apache-2.0 kept as `LICENSE-ioc-template`.
Nothing is copied from another driver: ArgyllCMS's `specbos.c` (GPL-2+) and
a Python PE60_2 integration were read as references for the command set.
