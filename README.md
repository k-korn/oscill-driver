# Oscill DSO1 plugin for ngscopeclient (Linux)

A libscopehal driver plugin for the OKB ROSA **Oscill DSO1** single-channel pen oscilloscope (CP210x USB-UART,
OBEX-style framing), reverse-engineered from the vendor's Windows software and verified on a real probe
(firmware 1.26).

* [docs/Protocol.md](docs/Protocol.md): the probe's protocol, registers and scaling, with what is verified on
  hardware and what comes from the vendor code.
* [docs/Driver.md](docs/Driver.md): how the driver works, building, testing.

## What works

Verified with ngscopeclient (scopehal-apps 0.3) and a real probe:

* identification, register access, link speed up to 921600 baud;
* volts/div 20 mV–10 V, sample rates 100 S/s–25 MS/s (volt and time scales checked within 1%);
* DC / AC / GND coupling, the probe's ~3 MHz and ~3 kHz low-pass filters as ADC modes (3 MHz also as the bandwidth limit);
* Run / Single / Auto / Force / Stop with the probe's **hardware edge trigger** (rising, falling, either; level and
  position), refined to sub-sample alignment in software.

Records are `QS` − 2 samples (252 with the power-up setting). Not supported: the probe's averaging / peak modes, RIS, ROLL, record length changes.

## Quick start

Build against the scopehal-apps version your ngscopeclient was built from (see
[docs/Driver.md](docs/Driver.md#building) for the full recipe, including distribution packages):

```bash
cmake -B build -DSCOPEHAL_APPS_DIR=/path/to/scopehal-apps -DSCOPEHAL_BUILD_DIR=/path/to/scopehal-apps/build
```

```bash
cmake --build build && cmake --install build
```

This installs `liboscill.so` into `~/.scopehal/plugins/`. In ngscopeclient add an oscilloscope with driver
**`oscill`**, transport **`uart`** and path **`/dev/ttyUSB0:921600`** (adjust the device). Your user needs access to
the serial port (`dialout` or `uucp` group, depending on the distribution).

If no `/dev/ttyUSB*` appears, check `lsusb`. For a `10c4:840e` adapter, bind the cp210x driver by hand:

```bash
echo 10c4 840e | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
```

If the probe stops answering after other software has opened the port, unplug and replug it: toggling DTR/RTS resets
it into its bootloader (details in [docs/Protocol.md](docs/Protocol.md#bootloader-boot-state)).
