"""
Motion 2350 Pro - CircuitPython hardware test platform.

Tests (in order on boot):
  1. RGB NeoPixel LEDs (GP23)
  2. DC motor pulse on all 4 channels
  3. USB gamepad scan + live control loop

Copy this project to your CIRCUITPY drive:
  - code.py          (auto-starts this test)
  - Test.py          (this file)
  - lib/             (see REQUIRED_LIBS below)

Flash CircuitPython first:
  https://circuitpython.org/board/cytron_motion_2350_pro/

Hold BOOTSEL, plug USB-C, drag the .uf2 onto the RPI-RP2 drive.

REQUIRED_LIBS (copy from the CircuitPython library bundle into CIRCUITPY/lib/):
  - adafruit_motor/motor.mpy
  - adafruit_usb_host_descriptors.mpy

neopixel is built into CircuitPython. adafruit_motor may already be in the
board firmware bundle — only copy libs if import fails.

USB HOST NOTES:
  - Plug the gamepad into the board's USB-A host port (not the USB-C PC port).
  - Power the board from battery/VIN when running motors; USB alone may not
    supply enough current.
  - RP2350 USB host can be picky with some gamepads. If nothing is detected,
    try a powered USB hub between the board and gamepad, or plug the pad in
    before powering the board on.
  - Cheap clone pads often use odd HID layouts; this code auto-calibrates on
    connect. Wiggle the left stick when the RGB turns purple.
  - Connect serial (Thonny / Mu / screen) at 115200 baud to see debug output.

On-board buttons during the gamepad loop:
  GP20 - re-run motor test
  GP21 - re-run RGB test
"""

import array
import time
import traceback

import board
import digitalio
import neopixel
import pwmio
import usb.core
import usb_host
from adafruit_motor import motor
import adafruit_usb_host_descriptors

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

MOTOR_PWM_FREQUENCY = 10000
MOTOR_TEST_THROTTLE = 0.35
MOTOR_TEST_DURATION_S = 0.6
MOTOR_DRIVE_THROTTLE = 0.45
JOYSTICK_DEADZONE = 15
GAMEPAD_SCAN_TIMEOUT_S = 30
GAMEPAD_READ_TIMEOUT_MS = 10
CALIBRATION_SECONDS = 6

# Fallback axis guesses for common cheap pads (unsigned 0-255, center 128).
CHEAP_AXIS_GUESSES = (
    (2, 3),
    (3, 4),
    (1, 2),
    (4, 5),
    (0, 1),
)

# RGB status colours (R, G, B)
COLOR_BOOT = (40, 0, 0)
COLOR_RGB_TEST = (0, 0, 80)
COLOR_MOTOR_TEST = (80, 40, 0)
COLOR_WAIT_GAMEPAD = (80, 80, 0)
COLOR_GAMEPAD_OK = (0, 80, 0)
COLOR_GAMEPAD_CALIBRATE = (60, 0, 60)
COLOR_GAMEPAD_ERROR = (80, 0, 0)
COLOR_IDLE = (0, 10, 10)

DIR_IN = 0x80

# ---------------------------------------------------------------------------
# Hardware setup
# ---------------------------------------------------------------------------

usb_host.Port(board.GP24, board.GP25)

pixels = neopixel.NeoPixel(board.GP23, 2, brightness=0.4, auto_write=True)

btn_rgb = digitalio.DigitalInOut(board.GP20)
btn_motor = digitalio.DigitalInOut(board.GP21)
for btn in (btn_rgb, btn_motor):
    btn.direction = digitalio.Direction.INPUT
    btn.pull = digitalio.Pull.UP


def _make_motors():
    pins = (
        (board.GP8, board.GP9),
        (board.GP10, board.GP11),
        (board.GP12, board.GP13),
        (board.GP14, board.GP15),
    )
    motors = []
    for pin_a, pin_b in pins:
        pwm_a = pwmio.PWMOut(pin_a, frequency=MOTOR_PWM_FREQUENCY)
        pwm_b = pwmio.PWMOut(pin_b, frequency=MOTOR_PWM_FREQUENCY)
        motors.append(motor.DCMotor(pwm_a, pwm_b))
    return motors


motors = _make_motors()


def set_rgb(color):
    pixels.fill(color)


def stop_motors():
    for m in motors:
        m.throttle = 0


def log(msg):
    print(msg)


def _button_pressed(btn):
    return not btn.value


def _hex_report(report, count=8):
    return " ".join("{:02x}".format(b) for b in report[:count])


def _axis_from_unsigned(raw_byte, center):
    value = int(raw_byte) - center
    if abs(value) < JOYSTICK_DEADZONE:
        return 0
    return max(-127, min(127, value))


def _axis_from_signed(raw_byte):
    value = raw_byte if raw_byte < 128 else raw_byte - 256
    if abs(value) < JOYSTICK_DEADZONE:
        return 0
    return max(-127, min(127, value))


class CheapGamepadReader:
    """Parse generic / clone USB gamepads with unknown HID layouts."""

    def __init__(self):
        self.idle = None
        self.report_len = 0
        self.report_id_offset = 0
        self.x_index = 2
        self.y_index = 3
        self.x_center = 128
        self.y_center = 128
        self.signed_axes = False
        self.calibrated = False

    def set_idle(self, report, count):
        self.idle = report[:count]
        self.report_len = count
        self.report_id_offset = 0
        if count > 4 and 0 < report[0] < 16:
            # Many cheap pads prefix reports with a report ID byte.
            self.report_id_offset = 1
        self._apply_best_guess()

    def _apply_best_guess(self):
        base = self.report_id_offset
        for x_idx, y_idx in CHEAP_AXIS_GUESSES:
            x_pos = base + x_idx
            y_pos = base + y_idx
            if y_pos < self.report_len:
                self.x_index = x_pos
                self.y_index = y_pos
                self.x_center = self.idle[x_pos]
                self.y_center = self.idle[y_pos]
                return

    def calibrate(self, device, endpoint_address, seconds=CALIBRATION_SECONDS):
        log("--- Cheap gamepad calibration ---")
        log("Wiggle the LEFT stick and tap the D-pad for {} seconds.".format(seconds))
        set_rgb(COLOR_GAMEPAD_CALIBRATE)

        buf = array.array("B", [0] * 64)
        peak_delta = [0] * 16
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline:
            try:
                count = device.read(endpoint_address, buf, GAMEPAD_READ_TIMEOUT_MS)
            except usb.core.USBTimeoutError:
                continue
            if count <= 0:
                continue

            for i in range(min(count, 16)):
                peak_delta[i] = max(peak_delta[i], abs(int(buf[i]) - int(self.idle[i])))

            time.sleep(0.01)

        ranked = sorted(range(16), key=lambda i: peak_delta[i], reverse=True)
        movers = [i for i in ranked if peak_delta[i] >= 20]

        if len(movers) >= 2:
            self.x_index, self.y_index = movers[0], movers[1]
            if self.x_index > self.y_index:
                self.x_index, self.y_index = self.y_index, self.x_index
            self.x_center = self.idle[self.x_index]
            self.y_center = self.idle[self.y_index]
            self.signed_axes = self._looks_signed(self.x_center, self.y_center)
            self.calibrated = True
            log(
                "Auto-detected stick on bytes {} and {} (center {}/{}).".format(
                    self.x_index, self.y_index, self.x_center, self.y_center
                )
            )
        else:
            log("No analog stick movement detected — using D-pad / button fallback.")
            log("Tap D-pad if motors do not respond to the stick.")

        pixels.fill(COLOR_GAMEPAD_OK)

    def _looks_signed(self, center_x, center_y):
        return center_x < 20 and center_y < 20

    def _drive_from_analog(self, report, count):
        if count <= max(self.x_index, self.y_index):
            return 0, 0

        if self.signed_axes:
            turn = _axis_from_signed(report[self.x_index])
            forward = -_axis_from_signed(report[self.y_index])
        else:
            turn = _axis_from_unsigned(report[self.x_index], self.x_center)
            forward = -_axis_from_unsigned(report[self.y_index], self.y_center)
        return forward, turn

    def _drive_from_dpad_snes(self, report, count):
        """SNES-clone style: byte0 = left/right, byte1 = up/down (0 or 0xFF)."""
        if count < 2:
            return 0, 0
        base = self.report_id_offset
        if base + 1 >= count:
            return 0, 0

        forward = 0
        turn = 0
        lr = report[base]
        ud = report[base + 1]

        if ud == 0:
            forward = 127
        elif ud == 0xFF:
            forward = -127

        if lr == 0:
            turn = -127
        elif lr == 0xFF:
            turn = 127

        return forward, turn

    def _drive_from_dpad_bits(self, report, count):
        """Many cheap pads pack D-pad in the low nibble of byte 0."""
        if count < 1:
            return 0, 0

        hat = report[self.report_id_offset] & 0x0F
        forward = 0
        turn = 0
        if hat == 0:
            pass
        else:
            # Standard HID hat: 1=E 2=NE 3=N 4=NW 5=W 6=SW 7=S 8=SE
            if hat in (3, 2, 4):
                forward = 127
            if hat in (7, 6, 8):
                forward = -127
            if hat in (5, 4, 6):
                turn = -127
            if hat in (1, 2, 8):
                turn = 127

        if forward or turn:
            return forward, turn

        # Bitmask style used by some clones (up/down/left/right in high nibble).
        bits = report[self.report_id_offset]
        if bits & 0x01:
            forward = 127
        if bits & 0x02:
            forward = -127
        if bits & 0x04:
            turn = -127
        if bits & 0x08:
            turn = 127

        if count > 1:
            bits = report[self.report_id_offset + 1]
            if bits & 0x01:
                forward = 127
            if bits & 0x02:
                forward = -127
            if bits & 0x04:
                turn = -127
            if bits & 0x08:
                turn = 127

        return forward, turn

    def _drive_from_dpad(self, report, count):
        fwd, turn = self._drive_from_dpad_snes(report, count)
        if fwd or turn:
            return fwd, turn
        return self._drive_from_dpad_bits(report, count)

    def get_drive(self, report, count):
        forward, turn = self._drive_from_analog(report, count)
        if forward or turn:
            return forward, turn
        return self._drive_from_dpad(report, count)


def test_rgb():
    log("--- RGB LED test ---")
    set_rgb(COLOR_RGB_TEST)
    sequence = (
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (255, 255, 0),
        (0, 255, 255),
        (255, 0, 255),
        (255, 255, 255),
    )
    for color in sequence:
        pixels[0] = color
        pixels[1] = color
        time.sleep(0.35)
    pixels.fill(0)
    log("RGB test complete.")


def test_motors():
    log("--- Motor test (wheels off ground!) ---")
    set_rgb(COLOR_MOTOR_TEST)

    log("Forward slow...")
    for m in motors:
        m.throttle = MOTOR_TEST_THROTTLE
    time.sleep(MOTOR_TEST_DURATION_S)
    stop_motors()
    time.sleep(0.4)

    log("Reverse slow...")
    for m in motors:
        m.throttle = -MOTOR_TEST_THROTTLE
    time.sleep(MOTOR_TEST_DURATION_S)
    stop_motors()
    time.sleep(0.4)

    log("Motor test complete.")
    set_rgb(COLOR_IDLE)


def _find_hid_endpoint(device):
    finders = (
        adafruit_usb_host_descriptors.find_gamepad_endpoint,
        adafruit_usb_host_descriptors.find_joystick_endpoint,
        adafruit_usb_host_descriptors.find_report_mouse_endpoint,
        adafruit_usb_host_descriptors.find_boot_mouse_endpoint,
    )
    for finder in finders:
        interface_index, endpoint_address = finder(device)
        if interface_index is not None and endpoint_address is not None:
            return interface_index, endpoint_address

    # Last resort: walk descriptors and grab the first HID IN endpoint.
    try:
        config_descriptor = adafruit_usb_host_descriptors.get_configuration_descriptor(
            device, 0
        )
    except usb.core.USBError:
        return None, None

    interface_index = None
    i = 0
    while i < len(config_descriptor):
        descriptor_len = config_descriptor[i]
        descriptor_type = config_descriptor[i + 1]
        if descriptor_type == adafruit_usb_host_descriptors.DESC_INTERFACE:
            interface_index = config_descriptor[i + 2]
            interface_class = config_descriptor[i + 5]
            if interface_class != 0x03:
                interface_index = None
        elif descriptor_type == adafruit_usb_host_descriptors.DESC_ENDPOINT:
            endpoint_address = config_descriptor[i + 2]
            if interface_index is not None and endpoint_address & DIR_IN:
                return interface_index, endpoint_address
        i += descriptor_len

    return None, None


def _print_device_info(device):
    try:
        log(
            "Device: VID={:04x} PID={:04x}  {} / {}".format(
                device.idVendor,
                device.idProduct,
                device.manufacturer,
                device.product,
            )
        )
    except usb.core.USBError:
        log(
            "Device: VID={:04x} PID={:04x}  (string descriptors unavailable)".format(
                device.idVendor, device.idProduct
            )
        )


def wait_for_gamepad(timeout_s=GAMEPAD_SCAN_TIMEOUT_S):
    log("--- USB gamepad scan ---")
    log("Plug a cheap wired pad into the USB-A host port.")
    log("Tip: plug it in BEFORE powering on if it is not detected.")
    set_rgb(COLOR_WAIT_GAMEPAD)

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for device in usb.core.find(find_all=True):
            _print_device_info(device)
            interface_index, endpoint_address = _find_hid_endpoint(device)
            if endpoint_address is None:
                log("  HID endpoint not found on this device, skipping.")
                continue

            try:
                device.set_configuration()
                if device.is_kernel_driver_active(interface_index):
                    device.detach_kernel_driver(interface_index)
            except usb.core.USBError as exc:
                log("  Could not configure device: {}".format(exc))
                continue

            log(
                "  Using interface {} endpoint 0x{:02x}".format(
                    interface_index, endpoint_address
                )
            )
            return device, endpoint_address

        time.sleep(0.25)

    return None, None


def _reports_differ(a, b, length):
    for i in range(length):
        if a[i] != b[i]:
            return True
    return False


def _apply_tank_drive(forward, turn):
    """Simple 2-motor tank drive on M1/M2; M3/M4 mirror for 4WD test rigs."""
    forward_f = max(-1.0, min(1.0, forward / 127.0))
    turn_f = max(-1.0, min(1.0, turn / 127.0))

    left = max(-1.0, min(1.0, forward_f + turn_f))
    right = max(-1.0, min(1.0, forward_f - turn_f))

    motors[0].throttle = left * MOTOR_DRIVE_THROTTLE
    motors[1].throttle = right * MOTOR_DRIVE_THROTTLE
    motors[2].throttle = left * MOTOR_DRIVE_THROTTLE
    motors[3].throttle = right * MOTOR_DRIVE_THROTTLE


def gamepad_control_loop(device, endpoint_address):
    log("--- Gamepad control loop ---")
    log("Left stick OR D-pad drives motors. GP20=motor test, GP21=RGB test.")
    set_rgb(COLOR_GAMEPAD_OK)

    pad = CheapGamepadReader()
    buf = array.array("B", [0] * 64)
    prev = None
    idle_logged = False
    calibration_done = False

    while True:
        if _button_pressed(btn_motor):
            stop_motors()
            time.sleep(0.2)
            test_motors()
            set_rgb(COLOR_GAMEPAD_OK)
            while not btn_motor.value:
                time.sleep(0.05)

        if _button_pressed(btn_rgb):
            stop_motors()
            time.sleep(0.2)
            test_rgb()
            set_rgb(COLOR_GAMEPAD_OK)
            while not btn_rgb.value:
                time.sleep(0.05)

        try:
            count = device.read(endpoint_address, buf, GAMEPAD_READ_TIMEOUT_MS)
        except usb.core.USBTimeoutError:
            continue
        except usb.core.USBError as exc:
            log("USB read error: {}".format(exc))
            set_rgb(COLOR_GAMEPAD_ERROR)
            time.sleep(1)
            return

        if count <= 0:
            continue

        if not idle_logged:
            pad.set_idle(buf, count)
            log("Idle report ({} bytes): {}".format(count, _hex_report(buf, min(count, 12))))
            idle_logged = True

        if not calibration_done:
            pad.calibrate(device, endpoint_address)
            calibration_done = True

        if prev is not None and _reports_differ(buf, prev, min(count, 12)):
            log("Report: {}".format(_hex_report(buf, min(count, 12))))

        forward, turn = pad.get_drive(buf, count)
        if forward or turn:
            _apply_tank_drive(forward, turn)
        elif prev is not None:
            stop_motors()

        prev = buf[:count]


def run_tests():
    log("")
    log("Motion 2350 Pro CircuitPython test platform")
    log("Board: {}".format(board.board_id))
    log("")

    set_rgb(COLOR_BOOT)
    time.sleep(0.5)

    try:
        test_rgb()
        test_motors()

        device, endpoint = wait_for_gamepad()
        if device is None:
            log("No gamepad found within timeout.")
            set_rgb(COLOR_GAMEPAD_ERROR)
            log("Idle — press GP20 for motor test, GP21 for RGB test.")
            while True:
                if _button_pressed(btn_motor):
                    test_motors()
                    while not btn_motor.value:
                        time.sleep(0.05)
                if _button_pressed(btn_rgb):
                    test_rgb()
                    while not btn_rgb.value:
                        time.sleep(0.05)
                time.sleep(0.05)
            return

        gamepad_control_loop(device, endpoint)

    except Exception as exc:
        traceback.print_exception(exc, exc, exc.__traceback__)
        set_rgb(COLOR_GAMEPAD_ERROR)
        stop_motors()
        raise
