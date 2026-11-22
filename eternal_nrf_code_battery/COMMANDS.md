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
cd ~/Desktop/eternal_nrf/eternal_nrf_code_battery
west build -b nrf52dk/nrf52810
```

Pristine (use after editing `prj.conf` / changing board):

```bash
west build -b nrf52dk/nrf52810 -p always
```

Output hex: `build/merged.hex` (ML621 cell curve + charging flag).

CR2032 build (coin-cell discharge curve, no charging flag) — used on the
main door board:

```bash
west build -b nrf52dk/nrf52810 -p always -d build_cr2032 -- -DEXTRA_CFLAGS=-DETERNAL_CELL_CR2032
```

Output hex: `build_cr2032/merged.hex`.

## Flash (OpenOCD + CMSIS-DAP)

One command: prints the device address, flashes, verifies, resets, and
clears the debug power-up latch so the board drops straight back to
sleep current (no power-cycle needed):

```bash
cd ~/Desktop/eternal_nrf/eternal_nrf_code_battery
openocd \
  -f interface/cmsis-dap.cfg \
  -f target/nrf52.cfg \
  -c "adapter speed 1000" \
  -c "init" \
  -c "mdw 0x100000A4 2" \
  -c "program build/merged.hex verify" \
  -c "reset run" \
  -c "nrf52.dap dpreg 4 0x00000000" \
  -c "echo [nrf52.dap dpreg 4]" \
  -c "exit"
```

- `adapter speed` must come **after** the target cfg, or it is overridden.
- **Device address**: `mdw 0x100000A4 2` prints two words `LO HI`. The BLE
  address is `(HI & 0xFFFF) | 0xC000` followed by `LO`, e.g.
  `dd049db8 cdda0b8a` → `CB:8A:DD:04:9D:B8`. Home Assistant names the
  entities after the last 4 hex digits (`eternal_9db8_*`).
- **Debug latch**: the last two commands write 0 to DP CTRL/STAT and read
  it back. `0x00000040` means CDBGPWRUPREQ/CSYSPWRUPREQ are cleared and
  the chip leaves debug-active mode (~1 mA) once the probe disconnects.
  If it still reads `0xf00000..`, power-cycle the board instead.
- **Erase failures** (`Error erasing sector N`, `Failed to read memory at
  0x4001e404`): usually a weak cell sagging during the erase. Retry at
  `adapter speed 100`; if it still fails, power the board from a bench
  supply on VCC (J1 pin 7) with JP1 open. A failed erase leaves the chip
  blank (`pc: 0xfffffffe`) until a flash completes.
- `clearing lockup after double fault` on a blank chip is expected.

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
probe-rs attach --chip nRF52810_xxAA build/eternal_nrf_code_battery/zephyr/zephyr.elf
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
  Clear the latch over SWD (`nrf52.dap dpreg 4 0x00000000`, see "Flash"
  above) or power-cycle the board.
- **OpenOCD `CMSIS-DAP command CMD_CONNECT failed` / pipe errors**:
  USB-side issue. Replug the probe, try a different USB cable, ensure
  the target rail is actually powered.
