# 01 — Raspberry Pi Pico 2 W

Board bring-up. Nothing else is worth debugging until this passes, because every
other test depends on being able to run code and read output.

## What you need

Just the Pico 2 W and a USB cable. Nothing else connected.

## Wiring

None. This test uses only the onboard LED.

| Signal | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| Onboard LED | `"LED"` | — | Behind the CYW43 WiFi chip on "W" boards, so it is **on/off only** and cannot be PWM'd |
| — | GP23, GP24, GP25, GP29 | 29, 31, 32, 34 | **Reserved.** Wired to the CYW43 chip, not free GPIO. Never assign these |

On a non-W Pico the LED is GP25 and *can* be PWM'd. `Pin("LED")` is the portable
name that works on both, which is why the code uses it.

## Flashing MicroPython

1. Download the UF2 for **Pico 2 W** from
   <https://micropython.org/download/RPI_PICO2_W/> — take the latest release.
2. Unplug the Pico.
3. Hold **BOOTSEL** down, plug the USB cable in, then release BOOTSEL.
4. A drive named `RP2350` appears. Copy the `.uf2` onto it:

   ```bash
   cp ~/Downloads/RPI_PICO2_W-*.uf2 /media/$USER/RP2350/
   sync
   ```

The board reboots the instant the copy completes and the drive vanishes. That is
success, not an error — ignore any "device removed unexpectedly" warning.

## Serial port permissions (Linux)

```bash
ls /dev/ttyACM*          # expect /dev/ttyACM0
sudo usermod -aG dialout $USER
```

Group membership is applied at login, so **log out and back in** afterwards.
To test without logging out, launch your editor through `sg`:

```bash
sg dialout -c thonny
```

A `[Errno 13] Permission denied: '/dev/ttyACM0'` in Thonny always means this,
never a wiring fault.

## Running the test

Open `test_pico_2w.py` in Thonny, set the interpreter to
**MicroPython (Raspberry Pi Pico)** in the bottom-right corner, and press Run.

## What you should see

The onboard LED blinks **three times slowly**, then **five times quickly**, and
then settles into a **steady pulse that stays lit for the rest of the run**. The
shell prints something like:

```
=== Pico 2 W bring-up ===
MicroPython : 3.4.0; MicroPython v1.24.1 on 2026-01-01 (RPi Pico 2 W with RP2350)
board       : RPi Pico 2 W with RP2350
CPU freq    : 150 MHz
free RAM    : 456384 bytes
reset cause : 1
LED         : 3 slow + 5 fast blinks - watch the board
LED         : now pulsing - stays lit while this script runs
WiFi chip   : present (CYW43 responded)

holding 8s so you can see the pulse...
=== all good ===
LED off - script ended
```

**Watch the board itself, not just the shell.** The blink pattern is proof that
code is genuinely executing on the hardware rather than the editor merely
connecting. The pulse that follows is the liveness heartbeat every other test in
this directory uses — this is your reference for what it should look like.

This script deliberately imports nothing from `src/`, because it runs before you
have uploaded any files to the board. Its heartbeat is therefore a hand-rolled
copy of `drivers/heartbeat.py`.

## If it fails

| Symptom | Cause |
|---|---|
| `Permission denied: '/dev/ttyACM0'` | dialout group — see above |
| No `/dev/ttyACM*` at all | MicroPython not flashed, or a charge-only USB cable |
| `WiFi chip: NOT detected` | You have a plain Pico 2, not a 2 W. Harmless here, but `Pin("LED")` behaves differently |
| Shell connects, LED never blinks | Board is in bootloader mode — unplug and replug without holding BOOTSEL |
