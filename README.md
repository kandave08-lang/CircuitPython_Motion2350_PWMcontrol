# CircuitPython Motion 2350 PWM Control

CircuitPython robotics controller for the **Cytron Motion 2350 Pro**. It drives brushed DC motors with PWM for remote car / bumper-bot control, and works with a USB gamepad or any radio transmitter and receiver wired to the board.

## Features

- Runs on power-up from `code.py` (no PC required after upload)
- RGB NeoPixel status lights
- 4-channel DC motor PWM test and drive
- USB host gamepad support (cheap/generic pads with auto-calibration)
- Works with radio TX/RX setups for car control

## Hardware

| Item | Notes |
|------|--------|
| Board | Cytron Motion 2350 Pro (RP2350) |
| Motors | Up to 4 brushed DC motors on M1–M4 |
| Power | Battery / VIN for motors (USB alone is often not enough) |
| Control | USB gamepad on the board USB-A host port, **or** any radio TX + RX connected for remote car control |

### Pin map (Motion 2350 Pro)

| Function | Pins |
|----------|------|
| Motor M1 | GP8 / GP9 |
| Motor M2 | GP10 / GP11 |
| Motor M3 | GP12 / GP13 |
| Motor M4 | GP14 / GP15 |
| RGB LEDs | GP23 |
| USB Host D+ / D− | GP24 / GP25 |
| User buttons | GP20, GP21 |

## Files

```
code.py   # CircuitPython entry point (auto-runs on boot)
Test.py   # RGB, motor, and USB gamepad test / drive logic
```

## Setup

1. Flash CircuitPython for this board:  
   [circuitpython.org/board/cytron_motion_2350_pro](https://circuitpython.org/board/cytron_motion_2350_pro/)
2. Copy `code.py` and `Test.py` to the root of the **CIRCUITPY** drive.
3. If imports fail, add these from the [CircuitPython library bundle](https://circuitpython.org/libraries) into `CIRCUITPY/lib/`:
   - `adafruit_motor/motor.mpy`
   - `adafruit_usb_host_descriptors.mpy`
4. Power the board from VIN/battery, plug a gamepad into **USB-A** (or connect your radio receiver), and turn the power switch **ON**.

Thonny is optional. You can copy files with File Explorer once **CIRCUITPY** appears.

## Boot sequence (`Test.py`)

1. RGB color cycle  
2. Short forward / reverse motor pulse (keep wheels off the ground)  
3. USB gamepad scan  
4. Purple LED = calibration — wiggle the left stick / tap the D-pad  
5. Stick or D-pad drives motors (tank-style on M1–M4)

**Buttons while running:** GP20 = re-run motor test · GP21 = re-run RGB test

## Radio / car control

This platform is meant for remote car control. You can:

- Use a **USB gamepad** on the Motion 2350 Pro host port, or  
- Connect **any radio transmitter + receiver** and map receiver PWM / channels into motor throttle the same way the gamepad drives the motors.

Cheap clone gamepads are supported via auto-calibration and D-pad fallbacks. If a pad is not detected, try plugging it in before power-on, or use a powered USB hub.

## License

Project code for personal / robotics use. Motion 2350 Pro hardware and examples are from [Cytron Technologies](https://www.cytron.io/p-motion-2350-pro).
