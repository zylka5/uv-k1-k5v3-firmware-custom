# Acurite 606TX receiver overlay app

This overlay app is a compact, API-v3-compatible proof-of-concept for the
Acurite 606TX temperature sensor on 433.92 MHz. It is structured like the other
app overlays in this repository and follows the same build rules.

## Notes

- It uses the existing demodulated audio capture path (`af_capture_open()`,
  `af_capture_sample()`) and listens for the OOK/ASK bursts used by Acurite
  devices.
- The code is ported from the rtl_433 Acurite family decoder logic and kept
  intentionally conservative to fit the 4 KiB overlay limit.
- The app is meant to be tuned against real captured packets on hardware, as the
  exact waveform and packet timing can vary slightly between sensor batches.

## Build

```sh
cd App/apps/acurite606tx
./build.sh
```

The resulting `.app` is packed with the same metadata as other overlays in the
repository.

## Usage

1. Tune the radio to the station frequency (the weather sensor is on 433.92 MHz).
2. Launch the app from the overlay menu.
3. The screen waits for a valid Acurite packet and then shows the latest ID,
   temperature, humidity and channel information.

This is a compact first implementation for the firmware; it is intentionally
small enough to stay within the overlay limit and can be refined once real
samples have been captured from a 606TX module.

## Known upstream reference

- `rtl_433` family decoder: `src/devices/acurite.c`
- Device family: Acurite 606TX / 5n1 style weather sensors
- Carrier: 433.92 MHz, OOK/ASK demodulated output

This port is not a full replacement for rtl_433's decoder, but it is usable as a
firmware-friendly starting point inside this overlay-app system.

