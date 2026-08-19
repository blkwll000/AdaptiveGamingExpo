"""

Live input monitor for the Orange Pi 4 Pro.

Wiring (physical pin numbers). Every module has its own VCC -> 3.3V and
GND -> GND, so only the signal pins land on the header:
    pin 8   KY-004 button A   OUT
    pin 10  KY-004 button B   OUT
    pin 19  KY-040 encoder    SW
    pin 21  KY-040 encoder    DT
    pin 23  KY-040 encoder    CLK
    pin 36  HW-504 joystick   SW
    pin 3   SDA (i2c-0)       -> ADS1115 + MPU
    pin 5   SCL (i2c-0)
    HW-504 VRx -> ADS1115 A3, VRy -> ADS1115 A2

Power the modules from 3.3V, not 5V - the GPIOs and the ADS inputs are 3.3V.

The KY-040 and joystick switches pull to GND when pressed, but KY-004 boards
come both ways depending on which side of the resistor the switch sits on. So
instead of assuming, every switch gets read once at startup and whatever level
it sits at is treated as "released".

`gpio readall` shows pins 8/10 as ALT8 (uart7) and 19/21/23 as ALT2 (spi3),
but those overlays are already off in armbian-config - it is just where the pin
mux registers sit by default. Since no driver is bound to them, the pinMode
calls below move the pins to plain input on their own, so there is nothing to
change in /boot/armbianEnv.txt. Leave the i2c0 overlay enabled, that one is
carrying the ADS and the MPU.

    sudo apt install python3-smbus2
    sudo python3 Desktop/inputTesting.py

wiringpi comes from wiringOP-Python, built from source. On trixie the stock
build breaks (SWIG 4.3 changed SWIG_Python_AppendOutput, GCC 14 errors on
implicit declarations), so patch before building:

    sudo apt install -y swig python3-dev python3-setuptools git
    git clone --recursive https://github.com/orangepi-xunlong/wiringOP-Python -b next
    cd wiringOP-Python
    git submodule update --init --remote
    sed -i 's/SWIG_Python_AppendOutput/SWIG_AppendOutput/g' wiringpi.i
    python3 generate-bindings.py > bindings.i
    sudo CFLAGS="-Wno-error=implicit-function-declaration" python3 setup.py install
"""

import curses
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

ADS_ADDR = 0x48
MPU_ADDR = 0x68

# full scale of the ADS at gain 1 (+/-4.096V) for a 3.3V joystick
ADS_MAX = 26400

SWITCHES = [BTN_A, BTN_B, ENC_SW, JOY_SW]

wiringpi.wiringPiSetupPhys()
for pin in [BTN_A, BTN_B, ENC_SW, ENC_DT, ENC_CLK, JOY_SW]:
    wiringpi.pinMode(pin, wiringpi.INPUT)
    wiringpi.pullUpDnControl(pin, wiringpi.PUD_UP)

time.sleep(0.05)  # let the pins settle before deciding what "released" looks like
rest = {p: wiringpi.digitalRead(p) for p in SWITCHES}

bus = SMBus(0)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # wake the MPU

position = 0
last_dir = 0
last_move = 0.0


def watch_encoder():
    """Poll the encoder much faster than the screen redraws so no steps get lost."""
    global position, last_dir, last_move
    clk_prev = wiringpi.digitalRead(ENC_CLK)
    while True:
        clk = wiringpi.digitalRead(ENC_CLK)
        if clk == 0 and clk_prev == 1:
            if wiringpi.digitalRead(ENC_DT) == 1:
                position += 1
                last_dir = 1
            else:
                position -= 1
                last_dir = -1
            last_move = time.time()
        clk_prev = clk
        time.sleep(0.001)


def read_ads(channel):
    """One single-shot conversion on a single-ended ADS1115 channel."""
    config = 0x8000  # start a conversion
    config |= (0x04 | channel) << 12  # single-ended mux
    config |= 0x0200  # +/-4.096V
    config |= 0x0100  # single shot
    config |= 0x00E0  # 860 SPS
    config |= 0x0003  # comparator off
    bus.write_i2c_block_data(ADS_ADDR, 0x01, [config >> 8, config & 0xFF])
    time.sleep(0.002)
    data = bus.read_i2c_block_data(ADS_ADDR, 0x00, 2)
    value = (data[0] << 8) | data[1]
    if value > 32767:
        value -= 65536
    return value


def center_bar(value, low, high, width):
    """Bar that fills outward from a middle tick, so a centered input looks centered."""
    mid = width // 2
    spot = int((value - low) / (high - low) * (width - 1))
    spot = max(0, min(width - 1, spot))
    cells = [" "] * width
    cells[mid] = "|"
    if spot > mid:
        for i in range(mid + 1, spot + 1):
            cells[i] = "#"
    if spot < mid:
        for i in range(spot, mid):
            cells[i] = "#"
    return "".join(cells)


def block(pressed):
    return "[####]" if pressed else "[    ]"


BOX_X = 44
BOX_Y = 9
BOX_W = 21
BOX_H = 7
SPINNER = "|/-\\"


def main(stdscr):
    global position

    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_CYAN, -1)
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    label = curses.color_pair(2) | curses.A_BOLD
    text = curses.color_pair(3)
    live = curses.color_pair(1) | curses.A_BOLD
    idle = curses.A_DIM

    rows, cols = stdscr.getmaxyx()
    if rows < 24 or cols < 80:
        stdscr.addstr(0, 0, "Need at least 80x24, this terminal is %dx%d" % (cols, rows))
        stdscr.getch()
        return

    prev = dict(rest)
    count = {p: 0 for p in SWITCHES}
    flash = {p: 0.0 for p in SWITCHES}

    threading.Thread(target=watch_encoder, daemon=True).start()

    # the static furniture only needs drawing once
    stdscr.erase()
    stdscr.addstr(0, 1, "ORANGE PI 4 PRO - INPUT MONITOR", curses.A_BOLD)
    stdscr.addstr(0, 58, "q quit   r reset", idle)
    stdscr.addstr(2, 1, "BUTTONS", label)
    stdscr.addstr(5, 1, "ENCODER", label)
    stdscr.addstr(9, 1, "JOYSTICK", label)
    stdscr.addstr(18, 1, "ACCEL 0x68", label)
    stdscr.addstr(BOX_Y, BOX_X, "+" + "-" * BOX_W + "+", idle)
    for i in range(BOX_H):
        stdscr.addstr(BOX_Y + 1 + i, BOX_X, "|" + " " * BOX_W + "|", idle)
    stdscr.addstr(BOX_Y + BOX_H + 1, BOX_X, "+" + "-" * BOX_W + "+", idle)

    while True:
        now = time.time()

        lit = {}
        for p in SWITCHES:
            reading = wiringpi.digitalRead(p)
            down = reading != rest[p]
            if down and prev[p] == rest[p]:
                count[p] += 1
                flash[p] = now
            prev[p] = reading
            # hold a quick tap on screen long enough to actually see it
            lit[p] = down or now - flash[p] < 0.12

        vrx = ADS_MAX - read_ads(3)  # HW-504 VRx is wired backwards relative to the display
        vry = read_ads(2)

        raw = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 6)
        accel = []
        for i in [0, 2, 4]:
            n = (raw[i] << 8) | raw[i + 1]
            if n > 32767:
                n -= 65536
            accel.append(n / 16384.0)
        # the MPU sits with X and Z swapped, and X comes out backwards
        accel = [-accel[2], accel[1], accel[0]]

        # buttons
        stdscr.addstr(3, 14, "pin 8  ", text)
        stdscr.addstr(3, 21, block(lit[BTN_A]), live if lit[BTN_A] else idle)
        stdscr.addstr(3, 28, "x%-5d" % count[BTN_A], text)
        stdscr.addstr(3, 40, "pin 10 ", text)
        stdscr.addstr(3, 47, block(lit[BTN_B]), live if lit[BTN_B] else idle)
        stdscr.addstr(3, 54, "x%-5d" % count[BTN_B], text)

        # encoder
        stdscr.addstr(6, 14, "pos %-6d" % position, text)
        stdscr.addstr(6, 25, SPINNER[position % 4], live if now - last_move < 0.4 else idle)
        if now - last_move < 0.4:
            stdscr.addstr(6, 28, "CW >> " if last_dir > 0 else "<< CCW", live)
        else:
            stdscr.addstr(6, 28, "      ")
        stdscr.addstr(6, 40, "SW pin19 ", text)
        stdscr.addstr(6, 49, block(lit[ENC_SW]), live if lit[ENC_SW] else idle)
        stdscr.addstr(6, 56, "x%-5d" % count[ENC_SW], text)
        stdscr.addstr(7, 14, "[" + center_bar(position, -20, 20, 33) + "]", text)

        # joystick
        stdscr.addstr(10, 14, "btn pin36 ", text)
        stdscr.addstr(10, 24, block(lit[JOY_SW]), live if lit[JOY_SW] else idle)
        stdscr.addstr(10, 31, "x%-5d" % count[JOY_SW], text)
        stdscr.addstr(11, 14, "VRX %-6d" % vrx, text)
        stdscr.addstr(11, 25, "[" + center_bar(vrx, 0, ADS_MAX, 13) + "]", text)
        stdscr.addstr(12, 14, "VRY %-6d" % vry, text)
        stdscr.addstr(12, 25, "[" + center_bar(vry, 0, ADS_MAX, 13) + "]", text)

        for i in range(BOX_H):
            stdscr.addstr(BOX_Y + 1 + i, BOX_X + 1, " " * BOX_W)
        mx = int(vrx / ADS_MAX * (BOX_W - 1))
        my = int((1 - vry / ADS_MAX) * (BOX_H - 1))
        mx = max(0, min(BOX_W - 1, mx))
        my = max(0, min(BOX_H - 1, my))
        stdscr.addstr(BOX_Y + 1 + my, BOX_X + 1 + mx, "o", live)

        # accelerometer
        for i, axis in enumerate("XYZ"):
            stdscr.addstr(19 + i, 14, "%s %+.2fg" % (axis, accel[i]), text)
            stdscr.addstr(19 + i, 25, "[" + center_bar(accel[i], -1.5, 1.5, 21) + "]", text)

        stdscr.refresh()

        key = stdscr.getch()
        if key == ord("q"):
            break
        if key == ord("r"):
            position = 0
            for p in SWITCHES:
                count[p] = 0

        time.sleep(0.03)


curses.wrapper(main)
bus.close()
