"""
Simple live input monitor for the Orange Pi 4 Pro.

Wiring (physical pin numbers), every module powered from 3.3V:
    pin 8   button A            pin 19  encoder SW
    pin 10  button B            pin 21  encoder DT
    pin 36  joystick button     pin 23  encoder CLK
    pin 3/5 SDA/SCL (i2c-0) -> ADS1115 + MPU6050
    joystick VRx -> ADS1115 A3, VRy -> ADS1115 A2

Run with:  sudo python3 inputSimple.py   (ctrl-c to quit)
"""

import threading
import time

import wiringpi
from smbus2 import SMBus

BTN_A = 8
BTN_B = 10
ENC_SW = 19
ENC_DT = 21
ENC_CLK = 23
JOY_SW = 36

ADS = 0x48
MPU = 0x68
ADS_MAX = 26400  # what the ADS reads at 3.3V

wiringpi.wiringPiSetupPhys()
for pin in [BTN_A, BTN_B, ENC_SW, ENC_DT, ENC_CLK, JOY_SW]:
    wiringpi.pinMode(pin, wiringpi.INPUT)
    wiringpi.pullUpDnControl(pin, wiringpi.PUD_UP)

# whatever level each switch sits at right now counts as "released"
time.sleep(0.05)
rest = {pin: wiringpi.digitalRead(pin) for pin in [BTN_A, BTN_B, ENC_SW, JOY_SW]}

bus = SMBus(0)
bus.write_byte_data(MPU, 0x6B, 0)  # wake up the MPU

position = 0


def watch_encoder():
    global position
    prev = wiringpi.digitalRead(ENC_CLK)
    while True:
        clk = wiringpi.digitalRead(ENC_CLK)
        if clk == 0 and prev == 1:  # falling edge = one click of the knob
            position += 1 if wiringpi.digitalRead(ENC_DT) else -1
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


def state(pin):
    return "DOWN" if wiringpi.digitalRead(pin) != rest[pin] else "up"


while True:
    x = ADS_MAX - read_ads(3)  # VRx is wired backwards, flip it
    y = read_ads(2)

    raw = bus.read_i2c_block_data(MPU, 0x3B, 6)
    accel = []
    for i in [0, 2, 4]:
        n = (raw[i] << 8) | raw[i + 1]
        if n > 32767:
            n -= 65536
        accel.append(n / 16384)
    # the MPU sits with X and Z swapped, and X comes out backwards
    accel = [-accel[2], accel[1], accel[0]]

    print("\033[H\033[J", end="")  # clear the screen
    print(f"buttons    A {state(BTN_A)}    B {state(BTN_B)}")
    print(f"encoder    pos {position}    button {state(ENC_SW)}")
    print(f"joystick   X {x:5d}   Y {y:5d}   button {state(JOY_SW)}")
    print(f"accel      X {accel[0]:+.2f}g   Y {accel[1]:+.2f}g   Z {accel[2]:+.2f}g")
    time.sleep(0.1)
