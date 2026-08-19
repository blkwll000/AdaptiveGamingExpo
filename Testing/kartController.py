"""
Virtual gamepad for SuperTuxKart, driven by the Orange Pi 4 Pro inputs.

Every physical input shows up as its natural gamepad control:

PHYSICAL INPUT                       GAMEPAD EVENT
joystick X / Y                       left stick
joystick press    (pin 36)           left stick click
encoder dial CW   (pins 21/23)       right trigger (8 clicks = fully pressed)
encoder dial CCW  (below zero)       left trigger  (8 clicks = fully pressed)
encoder press     (pin 19)           X button
button A          (pin 8)            A button
button B          (pin 10)           B button
accelerometer tilt X / Y             right stick

Nothing is tied to a game action here. Bind steering, accelerate,
brake/reverse, fire, nitro and skidding inside SuperTuxKart instead:
Options -> Controls -> "OrangePi Kart Controller", then wiggle the
input you want for each action.

Wiring is the same as inputSimple.py: joystick VRx/VRy on ADS1115
channels 3/2, MPU6050 on i2c-0, everything powered from 3.3V.

Setup:
    sudo apt install python3-evdev
    sudo python3 kartController.py     (ctrl-c to quit)
"""

import threading
import time

import wiringpi
from evdev import AbsInfo, UInput, ecodes as e
from smbus2 import SMBus

BTN_A_PIN = 8
BTN_B_PIN = 10
ENC_SW_PIN = 19
JOY_SW_PIN = 36
ENC_DT = 21
ENC_CLK = 23

ADS = 0x48
MPU = 0x68
ADS_MAX = 26400  # what the ADS reads at 3.3V
DIAL_MAX = 8     # encoder clicks each way before the dial axis maxes out

JOY_DEADZONE = 0.08   # stick readings smaller than this count as centered
TILT_DEADZONE = 0.05  # same idea for the accelerometer

wiringpi.wiringPiSetupPhys()
for pin in [BTN_A_PIN, BTN_B_PIN, ENC_SW_PIN, JOY_SW_PIN, ENC_DT, ENC_CLK]:
    wiringpi.pinMode(pin, wiringpi.INPUT)
    wiringpi.pullUpDnControl(pin, wiringpi.PUD_UP)

# whatever level each switch sits at right now counts as "released"
time.sleep(0.05)
rest = {pin: wiringpi.digitalRead(pin) for pin in [BTN_A_PIN, BTN_B_PIN, ENC_SW_PIN, JOY_SW_PIN]}

bus = SMBus(0)
bus.write_byte_data(MPU, 0x6B, 0)  # wake up the MPU

dial = 0  # encoder position, clamped to +/- DIAL_MAX clicks


def watch_encoder():
    global dial
    prev = wiringpi.digitalRead(ENC_CLK)
    while True:
        clk = wiringpi.digitalRead(ENC_CLK)
        if clk == 0 and prev == 1:  # falling edge = one click of the knob
            dial += 1 if wiringpi.digitalRead(ENC_DT) else -1
            dial = max(-DIAL_MAX, min(DIAL_MAX, dial))
        prev = clk
        time.sleep(0.001)


threading.Thread(target=watch_encoder, daemon=True).start()


def read_ads(channel):
    config = 0xC3E3 | (channel << 12)  # one-shot read, +/-4.096V range
    bus.write_i2c_block_data(ADS, 0x01, [config >> 8, config & 0xFF])
    time.sleep(0.002)
    hi, lo = bus.read_i2c_block_data(ADS, 0x00, 2)
    value = (hi << 8) | lo
    return value - 65536 if value > 32767 else value


def pressed(pin):
    return wiringpi.digitalRead(pin) != rest[pin]


def to_axis(value):
    return int(max(-1.0, min(1.0, value)) * 32767)


def deadzone(value, size):
    return 0.0 if abs(value) < size else value


def read_tilt():
    raw = bus.read_i2c_block_data(MPU, 0x3B, 6)
    accel = []
    for i in [0, 2, 4]:
        n = (raw[i] << 8) | raw[i + 1]
        if n > 32767:
            n -= 65536
        accel.append(n / 16384)  # in g, roughly -1..+1 when tilting
    # the MPU sits with X and Z swapped, and X comes out backwards
    return -accel[2], accel[1]  # forward = positive x, right = positive y


axis = AbsInfo(value=0, min=-32767, max=32767, fuzz=0, flat=0, resolution=0)
trigger = AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)
pad = UInput(
    {
        e.EV_KEY: [e.BTN_SOUTH, e.BTN_EAST, e.BTN_WEST, e.BTN_THUMBL],
        e.EV_ABS: [(e.ABS_X, axis), (e.ABS_Y, axis),
                   (e.ABS_RX, axis), (e.ABS_RY, axis),
                   (e.ABS_Z, trigger), (e.ABS_RZ, trigger)],
    },
    name="OrangePi Kart Controller",
)
print("gamepad created")
print("leave the controller flat on the table to zero the accelerometer...")
time.sleep(1)
zero_x, zero_y = read_tilt()
print("done - start SuperTuxKart and bind the controls\n")

while True:
    jx = 2 * (ADS_MAX - read_ads(3)) / ADS_MAX - 1  # VRx is wired backwards, flip it
    jy = 2 * read_ads(2) / ADS_MAX - 1
    jx = deadzone(jx, JOY_DEADZONE)
    jy = deadzone(jy, JOY_DEADZONE)

    tilt_x, tilt_y = read_tilt()
    tilt_x = deadzone(tilt_x - zero_x, TILT_DEADZONE)
    tilt_y = deadzone(tilt_y - zero_y, TILT_DEADZONE)

    gas = max(0, dial) * 255 // DIAL_MAX      # dial above zero presses the right trigger
    brake = max(0, -dial) * 255 // DIAL_MAX   # dial below zero presses the left trigger

    pad.write(e.EV_ABS, e.ABS_X, to_axis(jx))
    pad.write(e.EV_ABS, e.ABS_Y, to_axis(-jy))  # stick up = negative, like a real pad
    pad.write(e.EV_ABS, e.ABS_RZ, gas)
    pad.write(e.EV_ABS, e.ABS_Z, brake)
    pad.write(e.EV_ABS, e.ABS_RX, to_axis(tilt_y))   # tilt right = stick right
    pad.write(e.EV_ABS, e.ABS_RY, to_axis(-tilt_x))  # tilt forward = stick up
    pad.write(e.EV_KEY, e.BTN_SOUTH, pressed(BTN_A_PIN))
    pad.write(e.EV_KEY, e.BTN_EAST, pressed(BTN_B_PIN))
    pad.write(e.EV_KEY, e.BTN_WEST, pressed(ENC_SW_PIN))
    pad.write(e.EV_KEY, e.BTN_THUMBL, pressed(JOY_SW_PIN))
    pad.syn()

    print(f"\rstick {jx:+.2f} {jy:+.2f}   dial {dial:+3d}   tilt {tilt_x:+.2f} {tilt_y:+.2f}   "
          f"A{pressed(BTN_A_PIN):d} B{pressed(BTN_B_PIN):d} "
          f"enc{pressed(ENC_SW_PIN):d} joy{pressed(JOY_SW_PIN):d} ", end="")
    time.sleep(0.02)
