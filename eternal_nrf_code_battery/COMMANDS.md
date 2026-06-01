# Dev commands

Quick reference for building, flashing, and viewing logs on the
`nrf52dk/nrf52810` target with a WeAct CMSIS-DAP / DAPLink probe.

## Environment

NCS toolchain + Zephyr SDK paths (only needed if `west` complains about
missing toolchain / Zephyr SDK):

```bash
export PATH="/opt/nordic/ncs/toolchains/0c0f19d91c/bin:$PATH"
export ZEPHYR_BASE="/opt/nordic/ncs/v3.3.0/zephyr"
export ZEPHYR_SDK_INSTALL_DIR="/opt/nordic/ncs/toolchains/0c0f19d91c/opt/zephyr-sdk"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
```

## Build

Incremental:

```bash
cd ~/Desktop/eternal_nrf/eternal_nrf_code
west build -b nrf52dk/nrf52810
```

Pristine (use after editing `prj.conf` / changing board):

```bash
west build -b nrf52dk/nrf52810 -p always
```

Output hex: `build/merged.hex`.

## Flash (OpenOCD + CMSIS-DAP)

```bash
cd ~/Desktop/eternal_nrf/eternal_nrf_code
openocd \
  -f interface/cmsis-dap.cfg \
  -f target/nrf52.cfg \
  -c "program build/merged.hex verify reset exit"
```

**After every flash, power-cycle the board** (briefly disconnect VBAT /
battery / USB power) before measuring sleep current or deploying. The
SWD programming session leaves the nRF52 in "debug-active" mode with the
CPU clocked continuously (~1 mA draw) until POR clears the latched
`DBGPWRUPREQ` bit. Removing the probe alone is not enough.

## View RTT logs

> RTT is **disabled by default** in `prj.conf`
> (`CONFIG_USE_SEGGER_RTT=n`, `CONFIG_PRINTK=n`) for production power
> efficiency. To use the commands below, temporarily flip those four
> logging lines back to `=y`, rebuild, and re-flash:
>
> ```
> CONFIG_CONSOLE=y
> CONFIG_PRINTK=y
> CONFIG_USE_SEGGER_RTT=y
> CONFIG_RTT_CONSOLE=y
> ```

### Option A: probe-rs (single command, recommended)

```bash
brew install probe-rs-tools   # one-time
probe-rs attach --chip nRF52810_xxAA build/eternal_nrf_code/zephyr/zephyr.elf
```

Add `--reset` to restart the target on attach. `Ctrl-C` to detach.

### Option B: OpenOCD + netcat (two terminals)

Terminal 1 — start OpenOCD with an RTT server on port 9090:

```bash
openocd \
  -f interface/cmsis-dap.cfg \
  -f target/nrf52.cfg \
  -c "init" \
  -c "rtt setup 0x20000000 0x6000 \"SEGGER RTT\"" \
  -c "rtt start" \
  -c "rtt server start 9090 0"
```

Terminal 2 — attach to the stream:

```bash
nc localhost 9090
```

Optional: trigger a reset from a third terminal so you also see the
one-shot boot messages:

```bash
echo "reset run" | nc localhost 4444
```

## Verifying UICR / NFC pins

The build configures `nfct-pins-as-gpios` in the device tree so P0.09 +
P0.10 are freed from NFC mode. To confirm UICR.NFCPINS has actually been
written (bit 0 must be 0 = Disabled):

```bash
openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg \
  -c "init; mdw 0x1000120C 1; exit"
```

Expected output: `0x1000120c: 00000000` (or `fffffffe`). If you see
`0xffffffff`, the UICR write hasn't been applied — chip-erase and
re-flash:

```bash
openocd -f interface/cmsis-dap.cfg -f target/nrf52.cfg \
  -c "init; nrf52 mass_erase 0; exit"
```

## Troubleshooting

- **"No J-Link device found"** when using `west flash`: the default runner
  is `jlink`. Either invoke OpenOCD manually (above) or pass
  `--runner openocd`.
- **Device not advertising** on a board without a 32.768 kHz crystal:
  ensure `prj.conf` has
  `CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC=y` and
  `CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC_CALIBRATION=y` instead of the
  default `CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL=y`.
- **`nc localhost 9090` shows nothing**: nothing is wrong with RTT — the
  server only forwards what the target writes after you connect. Wait
  ≤ 2 s for the next periodic log line, or reset the target via the
  monitor port (`echo "reset run" | nc localhost 4444`).
- **Probe held by another process**: only one of OpenOCD / probe-rs can
  own the SWD bus at a time. `pkill openocd` before running probe-rs,
  and vice versa.
- **Board draws ~1 mA after flashing**: chip is in debug-active mode.
  Power-cycle the board (disconnect VBAT briefly). See the note under
  "Flash" above.
- **OpenOCD `CMSIS-DAP command CMD_CONNECT failed` / pipe errors**:
  USB-side issue. Replug the probe, try a different USB cable, ensure
  the target rail is actually powered.
