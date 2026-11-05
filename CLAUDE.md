# ioc-spectrometer

The spectrometer image of GEECS-BELLA's native EPICS IOCs: an areaDetector
driver (`ADSpecfirm`, `specfirm/`) for JETI SPECFIRM boards, the Thorlabs
CCT series' electronics, over an asyn octet port, on the `ioc-adaravis`
developer image R2-3ec3 (`Dockerfile`, as ioc-picoscope). What the driver
sends, and never sends: `README.md`.

- **Instances** live in GEECS-BELLA/geecs-epics, not here: `ccttest1`
  (`services/undulator/spectrometers/ccttest1`, profile `bench-cct`), and
  its twin on the fake (`hosts/sim-undulator/compose.yaml`, `sim/specfirm_sim.py`).
- **Why ours.** No EPICS driver for any JETI board exists (geecs-epics
  catalogue, 2026-11-05); the protocol is documented (JETI SPECFIRM manual
  rev. 170) and the board answers it over a plain CDC-ACM port, so the driver
  is written against the manual and the recorded transcripts, with no vendor
  library: the image may be published. Decision 14 puts spectrometers on
  areaDetector. Another spectrometer driver (an Ocean Optics, the older
  Thorlabs CCS) joins this image (decision 11: named by family).
- **The protocol is the bench's.** The driver follows what the unit
  answered, recorded in geecs-epics `services/undulator/spectrometers/ccttest1/`
  (the snapshot, the measurement probe, the fake). A behaviour the bench
  shows different is fixed in the driver and in the fake in the same change,
  and the instance README's "The protocol" section with it.
- **Images.** As ioc-daq: CI publishes on a git tag, CalVer `YYYY.M.N`
  (never re-push a tag), `ghcr.io/geecs-bella/ioc-spectrometer-runtime:<tag>`
  and `-developer`, made public once after the first tag. A pre-release tag
  (`2026.11.1b1`) for bench work from a branch. A local build:
  `DOCKER_DEFAULT_PLATFORM=linux/amd64 TARGET=runtime TAG=ioc-spectrometer:local ./build`.
- **Pins.** geecs-epics' compose files pin the tag; a bump is a tag here and
  one line there, then `pull <service>` and recreate on the host.
- **Merging**: after a fresh-context review dispositioned on the PR, and the
  bench proof for anything that changes what reaches a device. Rules and
  program state: geecs-epics `CLAUDE.md`.
