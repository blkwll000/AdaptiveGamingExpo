"""
Behind-the-scenes helpers for the accessible gaming controller workshop.

Participants never need to open this file. It hides the fiddly parts --
GPIO setup, i2c chatter with the ADS1115 and MPU6050, encoder polling
threads, the live dashboard, and the virtual gamepad -- behind a handful
of simple functions that the two lab notebooks call:

    connect(...)                   set up every pin and sensor, once
    read_button(pin)               True while a button is held
    read_joystick(side)            raw joystick numbers (0 .. ~26400)
    read_joystick_percent(side)    joystick as -1.0 .. +1.0
    read_dial(side)                encoder clicks since setup
    read_dial_percent(side)        encoder as -1.0 .. +1.0 (8 clicks each way)
    read_tilt()                    tilt in g: (forward, right)
    read_tilt_percent()            tilt as -1.0 .. +1.0, zeroed: (x, y)
    rescale(value, low, high)      squeeze any range into -1.0 .. +1.0
    deadzone(value, size)          snap tiny wobbles near center to 0
    remember_bounds(side, lo, hi)  store a joystick calibration
    remember_flat()                store the tilt zero point
    live_feed()                    dashboard that proves the inputs work
    make_gamepad()                 create the virtual gamepad
    left_stick(x, y)               move the gamepad's left stick
    right_stick(x, y)              move the gamepad's right stick
    trigger(side, amount)          squeeze a trigger, 0.0 .. 1.0
    button(name, pressed)          press/release A B X Y L R LS RS
    update()                       send everything to the game, ~50x/sec

Hardware notes live in inputTesting.py. Same wiring rules apply: every
module runs on 3.3V, joysticks go through the ADS1115 on i2c-0 (LEFT
joystick VRx->A3 VRy->A2, RIGHT joystick VRx->A1 VRy->A0), and the
MPU6050 tilt sensor shares the same i2c bus.
"""

import os
import threading
import time

import wiringpi
from smbus2 import SMBus
from IPython.display import clear_output

ADS_ADDR = 0x48
MPU_ADDR = 0x68
ADS_MAX = 26400   # what the ADS1115 reads with the joystick pushed to 3.3V
DIAL_MAX = 8      # encoder clicks each way before the axis maxes out
TILT_RANGE = 0.5  # tilting half a g (~30 degrees) counts as "all the way"
DEADZONE = 0.08

INPUT_TYPES = ("joystick", "encoder", "buttons", "tilt")

# Pin labels each input type must provide in its setup dict.
NEEDED_LABELS = {
    "joystick": ["SW"],
    "encoder": ["CLK", "DT", "SW"],
    "buttons": ["A", "B"],
    "tilt": [],
}

# Every usable GPIO physical pin on the Orange Pi 4 Pro 40-pin header,
# per `gpio readall`. Pins 3/5 are left out (they carry i2c-0 for the
# ADS1115 and MPU6050), and 26/27/28 are left out (claimed by other
# functions - PE4 sits in ALT2, PB5/PB4 in ALT8).
ALL_GPIO_PINS = [7, 8, 10, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24,
                 29, 31, 32, 33, 35, 36, 37, 38, 40]

# ADS1115 channels for each side's joystick: (VRx, VRy)
JOY_CHANNELS = {"left": (3, 2), "right": (1, 0)}

_config = {}       # side -> {"type": ..., "pins": {...}}
_rest = {}         # declared pin -> level when idle
_stray_rest = {}   # undeclared pin -> level when idle
_bounds = {"left": (0, ADS_MAX), "right": (0, ADS_MAX)}
_flat = [0.0, 0.0]  # tilt (forward, right) that counts as "not tilting"
_dial = {"left": 0, "right": 0}
_bus = None
_generation = 0    # bumped on every connect() so old encoder threads exit


def _fail(message):
    raise RuntimeError("\n\n" + message + "\n")


def connect(left_input, left_pins, right_input, right_pins):
    """Set up both declared inputs, the i2c sensors, and the stray-pin watch."""
    global _bus, _generation

    # wiringpi's C code calls exit(1) if it isn't root, which would kill the
    # whole notebook kernel with no useful message. Catch it here instead.
    if os.geteuid() != 0:
        _fail("This notebook's Python is not running with admin rights, so it "
              "can't touch the pins. Ask your facilitator to start Jupyter on "
              "the Pi with:  sudo jupyter notebook --allow-root --ip=0.0.0.0  "
              "and connect the notebook to that server.")

    left_input = str(left_input).strip().lower()
    right_input = str(right_input).strip().lower()
    for name in (left_input, right_input):
        if name not in INPUT_TYPES:
            _fail('"%s" is not an input I know. Pick one of: "joystick", '
                  '"encoder", "buttons", "tilt" (check the spelling!).' % name)
    if left_input == "tilt" and right_input == "tilt":
        _fail("Only one side can be the tilt sensor - there is just one "
              "accelerometer per controller. Pick something else for the other side.")

    _config["left"] = {"type": left_input, "pins": dict(left_pins)}
    _config["right"] = {"type": right_input, "pins": dict(right_pins)}

    # Make sure the pin dicts have exactly the labels their input type needs.
    used = []
    for side in ("left", "right"):
        kind = _config[side]["type"]
        pins = _config[side]["pins"]
        for label in NEEDED_LABELS[kind]:
            if label not in pins:
                _fail('Your %s input is a %s, so its pin list needs "%s". '
                      "Copy the matching example from the setup cell."
                      % (side.upper(), kind, label))
        for label, pin in pins.items():
            if label not in NEEDED_LABELS[kind]:
                _fail('Your %s %s has an extra pin called "%s" that I was not '
                      "expecting. Copy the matching example from the setup cell."
                      % (side.upper(), kind, label))
            if pin not in ALL_GPIO_PINS:
                _fail("Pin %s (%s %s) is not a pin you can use. Usable pins: %s. "
                      "Remember these are the PHYSICAL pin numbers - count them "
                      "on the header." % (pin, side.upper(), label, ALL_GPIO_PINS))
            if pin in used:
                _fail("Pin %s is listed twice in your setup. Two wires cannot "
                      "share one pin - double check your numbers." % pin)
            used.append(pin)

    print("setting up your controller...")
    for side in ("left", "right"):
        kind = _config[side]["type"]
        pins = _config[side]["pins"]
        if pins:
            where = ", ".join("%s -> pin %d" % (label, pin) for label, pin in pins.items())
        else:
            where = "talks over the i2c wires on pins 3 and 5"
        print("  %-5s = %-8s (%s)" % (side.upper(), kind, where))

    _generation += 1  # any encoder threads from a previous run stop themselves

    wiringpi.wiringPiSetupPhys()
    for pin in ALL_GPIO_PINS:
        wiringpi.pinMode(pin, wiringpi.INPUT)
        wiringpi.pullUpDnControl(pin, wiringpi.PUD_UP)
    time.sleep(0.05)

    # Whatever level each pin sits at right now counts as "nothing happening".
    # This handles KY-004 boards that come wired either way around.
    _rest.clear()
    _stray_rest.clear()
    for pin in ALL_GPIO_PINS:
        level = wiringpi.digitalRead(pin)
        if pin in used:
            _rest[pin] = level
        else:
            _stray_rest[pin] = level

    types = (left_input, right_input)
    if "joystick" in types or "tilt" in types:
        _bus = SMBus(0)

    if "joystick" in types:
        try:
            _bus.read_byte(ADS_ADDR)
            print("  found the ADS1115 (the chip that reads the joystick) - good!")
        except OSError:
            _fail("I can't find the ADS1115 - the little chip that reads the "
                  "joystick. Check its wires: VDD -> 3.3V, GND -> GND, "
                  "SDA -> pin 3, SCL -> pin 5.")

    if "tilt" in types:
        try:
            _bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # wake it up
        except OSError:
            _fail("I can't find the MPU6050 tilt sensor. Check its wires: "
                  "VCC -> 3.3V, GND -> GND, SDA -> pin 3, SCL -> pin 5.")
        print("  found the MPU6050 tilt sensor - hold the controller flat and still...")
        time.sleep(1.0)
        remember_flat()
        print("  tilt zeroed! this position now counts as 'not tilting'.")

    _dial["left"] = 0
    _dial["right"] = 0
    for side in ("left", "right"):
        if _config[side]["type"] == "encoder":
            pins = _config[side]["pins"]
            threading.Thread(target=_watch_encoder,
                             args=(side, pins["CLK"], pins["DT"], _generation),
                             daemon=True).start()

    print("all set! I'm also quietly watching the other %d header pins in case "
          "a wire landed in the wrong hole." % len(_stray_rest))


def _watch_encoder(side, clk_pin, dt_pin, generation):
    """Poll the encoder far faster than any display loop so no clicks get lost."""
    prev = wiringpi.digitalRead(clk_pin)
    while generation == _generation:
        clk = wiringpi.digitalRead(clk_pin)
        if clk == 0 and prev == 1:  # falling edge = one click of the knob
            _dial[side] += 1 if wiringpi.digitalRead(dt_pin) else -1
        prev = clk
        time.sleep(0.001)


def _check_side(side, wanted):
    if side not in ("left", "right"):
        _fail('The side must be "left" or "right", not "%s".' % side)
    if not _config:
        _fail("Nothing is set up yet - run the ws.connect(...) cell first.")
    kind = _config[side]["type"]
    if kind != wanted:
        other = "right" if side == "left" else "left"
        hint = ""
        if _config[other]["type"] == wanted:
            hint = ' Your %s IS a %s - try "%s" instead.' % (other.upper(), wanted, other)
        _fail("Your %s input is set to '%s', not '%s'.%s"
              % (side.upper(), kind, wanted, hint))


# ---------------------------------------------------------------- reading

def read_button(pin):
    """True while the switch on this pin is held down."""
    if pin not in _rest:
        _fail("Pin %s is not one of the pins in your setup cell (you told me: %s). "
              "Fix the number, or re-run the setup and connect cells."
              % (pin, sorted(_rest) if _rest else "nothing yet - run ws.connect first"))
    return wiringpi.digitalRead(pin) != _rest[pin]


def _read_ads(channel):
    config = 0xC3E3 | (channel << 12)  # one-shot read, +/-4.096V range
    _bus.write_i2c_block_data(ADS_ADDR, 0x01, [config >> 8, config & 0xFF])
    time.sleep(0.002)
    hi, lo = _bus.read_i2c_block_data(ADS_ADDR, 0x00, 2)
    value = (hi << 8) | lo
    return value - 65536 if value > 32767 else value


def read_joystick(side):
    """Raw joystick position: (x, y), each roughly 0 .. 26400, center ~13000."""
    _check_side(side, "joystick")
    cx, cy = JOY_CHANNELS[side]
    x = ADS_MAX - _read_ads(cx)  # VRx on the HW-504 is wired backwards, flip it
    y = _read_ads(cy)
    return x, y


def read_joystick_percent(side):
    """Joystick as (x, y) in -1.0 .. +1.0 using the remembered bounds, deadzoned."""
    raw_x, raw_y = read_joystick(side)
    low, high = _bounds[side]
    x = deadzone(rescale(raw_x, low, high), DEADZONE)
    y = deadzone(rescale(raw_y, low, high), DEADZONE)
    return x, y


def read_dial(side):
    """Encoder position in clicks: turning one way counts up, the other way down."""
    _check_side(side, "encoder")
    return _dial[side]


def read_dial_percent(side):
    """Encoder as -1.0 .. +1.0, maxing out at 8 clicks either way."""
    return max(-1.0, min(1.0, read_dial(side) / DIAL_MAX))


def read_tilt():
    """Tilt in g as (forward, right). Flat on the table reads about (-1, 0)."""
    if "tilt" not in (_config.get("left", {}).get("type"), _config.get("right", {}).get("type")):
        _fail('Neither side of your controller is set to "tilt". '
              "Check your setup cell, then re-run ws.connect(...).")
    raw = _bus.read_i2c_block_data(MPU_ADDR, 0x3B, 6)
    accel = []
    for i in (0, 2, 4):
        n = (raw[i] << 8) | raw[i + 1]
        if n > 32767:
            n -= 65536
        accel.append(n / 16384)
    # this MPU sits with X and Z swapped, and X comes out backwards
    return -accel[2], accel[1]  # (forward, right)


def read_tilt_percent():
    """Tilt as a stick position (x, y): tilt right = +x, tilt forward = +y."""
    forward, right = read_tilt()
    x = (right - _flat[1]) / TILT_RANGE
    y = (forward - _flat[0]) / TILT_RANGE
    x = deadzone(max(-1.0, min(1.0, x)), DEADZONE)
    y = deadzone(max(-1.0, min(1.0, y)), DEADZONE)
    return x, y


# ---------------------------------------------------------------- bounds

def rescale(value, low, high):
    """Squeeze a number from the range low..high into -1.0 .. +1.0."""
    scaled = 2 * (value - low) / (high - low) - 1
    return max(-1.0, min(1.0, scaled))


def deadzone(value, size=DEADZONE):
    """Snap values near zero to exactly zero, so a resting input stays still."""
    return 0.0 if abs(value) < size else value


def remember_bounds(side, low, high):
    """Store a joystick's measured low/high so percent reads use them from now on."""
    _check_side(side, "joystick")
    if high - low < 1000:
        _fail("Those bounds are too close together (%s to %s) - the joystick "
              "probably didn't move during calibration. Run the wiggle cell "
              "again and really move it around!" % (low, high))
    _bounds[side] = (low, high)
    print("remembered! your %s joystick reads %d (one end) to %d (other end)."
          % (side, low, high))


def remember_flat():
    """Record the current tilt as the zero point. Hold the controller flat first!"""
    samples = [read_tilt() for _ in range(10)]
    _flat[0] = sum(s[0] for s in samples) / len(samples)
    _flat[1] = sum(s[1] for s in samples) / len(samples)


# ---------------------------------------------------------------- live feed

def _bar(value, width=13):
    """A little meter like [   #  |      ] for a value in -1..+1."""
    cells = [" "] * width
    cells[width // 2] = "|"
    spot = int((value + 1) / 2 * (width - 1))
    cells[max(0, min(width - 1, spot))] = "#"
    return "[" + "".join(cells) + "]"


def _switch_pins(side):
    """The press-type pins for a side (encoder CLK/DT are not presses)."""
    pins = _config[side]["pins"]
    return {label: pin for label, pin in pins.items() if label in ("SW", "A", "B")}


def live_feed():
    """Full-screen dashboard: every input live, with WORKING checks and a
    watch on all the pins the participant did NOT declare."""
    if not _config:
        _fail("Nothing is set up yet - run the setup and ws.connect(...) cells first.")

    counts = {}
    was_down = {}
    for side in ("left", "right"):
        for pin in _switch_pins(side).values():
            counts[pin] = 0
            was_down[pin] = False

    seen = {"left": set(), "right": set()}
    start_dial = dict(_dial)
    stray_was = {pin: False for pin in _stray_rest}
    stray_counts = {}
    stray_pin = None
    stray_time = 0.0
    my_pins = sorted(_rest)

    try:
        while True:
            now = time.time()

            for pin in counts:
                down = read_button(pin)
                if down and not was_down[pin]:
                    counts[pin] += 1
                was_down[pin] = down

            for pin, rest_level in _stray_rest.items():
                active = wiringpi.digitalRead(pin) != rest_level
                if active and not stray_was[pin]:
                    stray_counts[pin] = stray_counts.get(pin, 0) + 1
                    stray_pin, stray_time = pin, now
                stray_was[pin] = active

            lines = ["LIVE INPUT TEST - wiggle, press and turn everything!",
                     "(hit the square stop button next to the cell when you're done)",
                     ""]

            for side in ("left", "right"):
                kind = _config[side]["type"]
                pins = _config[side]["pins"]
                detail = []

                if kind == "joystick":
                    x, y = read_joystick_percent(side)
                    if abs(x) > 0.5 or abs(y) > 0.5:
                        seen[side].add("move")
                    if counts[pins["SW"]] > 0:
                        seen[side].add("press")
                    needed = {"move", "press"}
                    detail.append("   X %s %+.2f    Y %s %+.2f   %s"
                                  % (_bar(x), x, _bar(y), y,
                                     "OK" if "move" in seen[side] else "<- move it!"))
                    detail.append("   press (pin %d)  %s  pressed x%-3d %s"
                                  % (pins["SW"],
                                     "[####]" if was_down[pins["SW"]] else "[    ]",
                                     counts[pins["SW"]],
                                     "OK" if "press" in seen[side] else "<- press it!"))

                elif kind == "encoder":
                    pos = _dial[side]
                    pct = read_dial_percent(side)
                    if pos != start_dial[side]:
                        seen[side].add("turn")
                    if counts[pins["SW"]] > 0:
                        seen[side].add("press")
                    needed = {"turn", "press"}
                    detail.append("   dial %s %+3d clicks   %s"
                                  % (_bar(pct), pos,
                                     "OK" if "turn" in seen[side] else "<- turn it!"))
                    detail.append("   press (pin %d)  %s  pressed x%-3d %s"
                                  % (pins["SW"],
                                     "[####]" if was_down[pins["SW"]] else "[    ]",
                                     counts[pins["SW"]],
                                     "OK" if "press" in seen[side] else "<- press it!"))

                elif kind == "buttons":
                    for label in ("A", "B"):
                        if counts[pins[label]] > 0:
                            seen[side].add(label)
                        detail.append("   button %s (pin %d)  %s  pressed x%-3d %s"
                                      % (label, pins[label],
                                         "[####]" if was_down[pins[label]] else "[    ]",
                                         counts[pins[label]],
                                         "OK" if label in seen[side] else "<- press it!"))
                    needed = {"A", "B"}

                else:  # tilt
                    x, y = read_tilt_percent()
                    if abs(x) > 0.5 or abs(y) > 0.5:
                        seen[side].add("tilt")
                    needed = {"tilt"}
                    detail.append("   X %s %+.2f    Y %s %+.2f   %s"
                                  % (_bar(x), x, _bar(y), y,
                                     "OK" if "tilt" in seen[side] else "<- tilt it!"))

                status = "ALL WORKING!" if seen[side] >= needed else "not verified yet..."
                lines.append("%-5s  %-9s %s" % (side.upper(), kind, status))
                lines.extend(detail)
                lines.append("")

            if stray_pin is not None and now - stray_time < 3.0:
                lines.append("!! ACTIVITY ON PIN %d - that is NOT one of your pins (%s)."
                             % (stray_pin, my_pins))
                lines.append("!! A wire may be in the wrong hole. Check your setup cell numbers!")
            if stray_counts:
                lines.append("(stray pins seen this session: %s)"
                             % ", ".join("pin %d x%d" % (p, c)
                                         for p, c in sorted(stray_counts.items())))

            clear_output(wait=True)
            print("\n".join(lines))
            time.sleep(0.08)

    except KeyboardInterrupt:
        clear_output(wait=True)
        print("live test stopped.")
        if stray_counts:
            print("note: I saw activity on pins that are not in your setup: %s"
                  % ", ".join("pin %d" % p for p in sorted(stray_counts)))
            print("if an input never showed up above, its wire is probably on one of those pins.")


# ---------------------------------------------------------------- gamepad

_pad = None
_e = None
_btn_codes = {}
_sent = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "lt": 0.0, "rt": 0.0,
         "buttons": set()}
_last_status = 0.0


def make_gamepad():
    """Create the virtual gamepad. Games will see 'Workshop Controller'."""
    global _pad, _e, _btn_codes

    if not os.path.exists("/dev/uinput"):
        os.system("modprobe uinput")  # try to load the kernel module ourselves
        time.sleep(0.3)
    if not os.path.exists("/dev/uinput"):
        _fail("The computer's 'pretend gamepad' feature (/dev/uinput) is not "
              "switched on. Open a terminal and run:  sudo modprobe uinput  "
              "then run this cell again.")

    from evdev import AbsInfo, UInput, ecodes as e
    _e = e
    _btn_codes = {"A": e.BTN_SOUTH, "B": e.BTN_EAST, "X": e.BTN_WEST,
                  "Y": e.BTN_NORTH, "L": e.BTN_TL, "R": e.BTN_TR,
                  "LS": e.BTN_THUMBL, "RS": e.BTN_THUMBR}

    axis = AbsInfo(value=0, min=-32767, max=32767, fuzz=0, flat=0, resolution=0)
    trig = AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)
    _pad = UInput(
        {
            e.EV_KEY: list(_btn_codes.values()),
            e.EV_ABS: [(e.ABS_X, axis), (e.ABS_Y, axis),
                       (e.ABS_RX, axis), (e.ABS_RY, axis),
                       (e.ABS_Z, trig), (e.ABS_RZ, trig)],
        },
        name="Workshop Controller",
    )
    print("virtual gamepad created! games will now see a controller")
    print('called "Workshop Controller" - just like a USB gamepad was plugged in.')


def _need_pad():
    if _pad is None:
        _fail("There is no gamepad yet - run the ws.make_gamepad() cell first.")


def _to_axis(value):
    return int(max(-1.0, min(1.0, value)) * 32767)


def left_stick(x, y):
    """Move the gamepad's left stick. x: -1 left .. +1 right, y: -1 down .. +1 up."""
    _need_pad()
    _pad.write(_e.EV_ABS, _e.ABS_X, _to_axis(x))
    _pad.write(_e.EV_ABS, _e.ABS_Y, _to_axis(-y))  # real pads report up as negative
    _sent["lx"], _sent["ly"] = x, y


def right_stick(x, y):
    """Move the gamepad's right stick. Same directions as left_stick."""
    _need_pad()
    _pad.write(_e.EV_ABS, _e.ABS_RX, _to_axis(x))
    _pad.write(_e.EV_ABS, _e.ABS_RY, _to_axis(-y))
    _sent["rx"], _sent["ry"] = x, y


def trigger(side, amount):
    """Squeeze the left or right trigger: 0.0 = released, 1.0 = fully pressed."""
    _need_pad()
    amount = max(0.0, min(1.0, amount))
    if side == "left":
        _pad.write(_e.EV_ABS, _e.ABS_Z, int(amount * 255))
        _sent["lt"] = amount
    elif side == "right":
        _pad.write(_e.EV_ABS, _e.ABS_RZ, int(amount * 255))
        _sent["rt"] = amount
    else:
        _fail('trigger() needs "left" or "right", not "%s".' % side)


def button(name, pressed):
    """Press (True) or release (False) a gamepad button: A B X Y L R LS RS."""
    _need_pad()
    name = str(name).upper()
    if name not in _btn_codes:
        _fail('"%s" is not a gamepad button I know. Pick one of: %s'
              % (name, " ".join(_btn_codes)))
    _pad.write(_e.EV_KEY, _btn_codes[name], 1 if pressed else 0)
    if pressed:
        _sent["buttons"].add(name)
    else:
        _sent["buttons"].discard(name)


def update():
    """Send everything to the game, show a one-line status, and pace the loop."""
    global _last_status
    _need_pad()
    _pad.syn()
    now = time.time()
    if now - _last_status > 0.15:
        _last_status = now
        held = " ".join(sorted(_sent["buttons"])) or "-"
        print("\rsticks  L(%+.2f, %+.2f)  R(%+.2f, %+.2f)   triggers %.2f / %.2f   held: %-12s"
              % (_sent["lx"], _sent["ly"], _sent["rx"], _sent["ry"],
                 _sent["lt"], _sent["rt"], held), end="")
    time.sleep(0.02)
