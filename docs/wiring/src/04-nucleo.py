#!/usr/bin/env python3
"""Generate docs/wiring/04-nucleo.svg: Nucleo-F401RE to ultrasonics, servos, battery sense.

Run from anywhere (standard library only):

    python3 docs/wiring/src/04-nucleo.py

Header data follows ST UM1724 Rev 17, Figure 17 (NUCLEO-F401RE pinout) and
Tables 16/29 (Arduino and Morpho connectors). The pin assignment is the one the
STM32 firmware uses; change it here and in docs/wiring.md together.

Layout: board drawn top view, ST-LINK USB end up. Left Arduino headers (CN6,
CN8) wire out to the left, right headers (CN5, CN9) to the right. +5V and GND
are drawn as flags that all tie to the rails on the interface perfboard; with
the HC-SR04 pin order (VCC, TRIG, ECHO, GND) that is the only way to draw the
harness without crossings.
"""

from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "04-nucleo.svg"
WIDTH, HEIGHT = 1600, 1110
PITCH = 26

# (pin, Arduino name, MCU port, Morpho position), listed top to bottom as drawn.
CN6 = [(1, "NC", "", "CN7-10"), (2, "IOREF", "", "CN7-12"), (3, "RESET", "", "CN7-14"),
       (4, "+3V3", "", "CN7-16"), (5, "+5V", "", "CN7-18"), (6, "GND", "", "CN7-20"),
       (7, "GND", "", "CN7-22"), (8, "VIN", "", "CN7-24")]
CN8 = [(1, "A0", "PA0", "CN7-28"), (2, "A1", "PA1", "CN7-30"), (3, "A2", "PA4", "CN7-32"),
       (4, "A3", "PB0", "CN7-34"), (5, "A4", "PC1", "CN7-36"), (6, "A5", "PC0", "CN7-38")]
CN5 = [(10, "D15", "PB8", "CN10-3"), (9, "D14", "PB9", "CN10-5"), (8, "AVDD", "", "CN10-7"),
       (7, "GND", "", "CN10-9"), (6, "D13", "PA5", "CN10-11"), (5, "D12", "PA6", "CN10-13"),
       (4, "D11", "PA7", "CN10-15"), (3, "D10", "PB6", "CN10-17"), (2, "D9", "PC7", "CN10-19"),
       (1, "D8", "PA9", "CN10-21")]
CN9 = [(8, "D7", "PA8", "CN10-23"), (7, "D6", "PB10", "CN10-25"), (6, "D5", "PB4", "CN10-27"),
       (5, "D4", "PB5", "CN10-29"), (4, "D3", "PB3", "CN10-31"), (3, "D2", "PA10", "CN10-33"),
       (2, "D1", "PA2", "CN10-35"), (1, "D0", "PA3", "CN10-37")]

# Wire colours (also used for the pin highlight).
C = {
    "v5": "#d32f2f", "gnd": "#212121", "trig": "#f9a825", "echo": "#2e7d32",
    "sense": "#7b1fa2", "sig": "#ef6c00", "vplus": "#d32f2f", "vminus": "#6d4c41",
    "usb": "#1565c0",
}

# Used pins: (connector, pin) -> wire colour key.
USED = {
    ("CN6", 5): "v5", ("CN6", 6): "gnd",
    ("CN8", 1): "trig", ("CN8", 2): "echo", ("CN8", 3): "sense",
    ("CN5", 7): "gnd", ("CN5", 5): "sig", ("CN5", 4): "sig", ("CN5", 1): "echo",
    ("CN9", 8): "trig",
}

# Pins that are taken on the board itself and must stay unwired.
RESERVED = {("CN5", 6), ("CN9", 2), ("CN9", 1)}  # D13 = LD2; D1/D0 = ST-LINK USB serial

# Header geometry: x of the pin column and y of the first (top) pin.
LEFT_X, RIGHT_X = 620, 980
Y0 = {"CN5": 340, "CN9": 614, "CN6": 444, "CN8": 666}

# ECHO dividers: y of the divider node (2 kOhm to GND) and of the 1 kOhm body below
# it. The node must sit between the Nucleo pin (above) and the 1 kOhm (below).
DIV_NODE_Y = 770
DIV_1K_Y1, DIV_1K_Y2 = 810, 866

out = []


def add(s):
    out.append(s)


def pin_y(conn, pin):
    rows = {"CN5": CN5, "CN9": CN9, "CN6": CN6, "CN8": CN8}[conn]
    index = [r[0] for r in rows].index(pin)
    return Y0[conn] + index * PITCH


def text(x, y, s, cls="s", anchor=None, fill=None):
    a = f' text-anchor="{anchor}"' if anchor else ""
    f = f' fill="{fill}"' if fill else ""
    add(f'<text x="{x}" y="{y}" class="{cls}"{a}{f}>{s}</text>')


def wire(d, colour, width=3.2):
    add(f'<path d="{d}" fill="none" stroke="{colour}" stroke-width="{width}" '
        f'stroke-linejoin="round"/>')


def dot(x, y, colour="#212121"):
    add(f'<circle cx="{x}" cy="{y}" r="4.5" fill="{colour}"/>')


def res_v(x, y1, y2):
    """Resistor body on a vertical wire (covers the wire underneath)."""
    add(f'<rect x="{x - 8}" y="{y1}" width="16" height="{y2 - y1}" fill="#ffffff" '
        f'stroke="#333" stroke-width="1.8"/>')


def res_h(x1, x2, y):
    add(f'<rect x="{x1}" y="{y - 8}" width="{x2 - x1}" height="16" fill="#ffffff" '
        f'stroke="#333" stroke-width="1.8"/>')


def flag(x, y, label, colour):
    """Power flag centred on x, bottom edge at y."""
    w = 48
    add(f'<rect x="{x - w / 2}" y="{y - 20}" width="{w}" height="20" rx="4" fill="#ffffff" '
        f'stroke="{colour}" stroke-width="2"/>')
    text(x, y - 5, label, "s b", "middle", colour)


def gnd_symbol(x, y):
    """Ground symbol hanging from (x, y)."""
    add(f'<path d="M{x} {y} V{y + 10} M{x - 12} {y + 10} H{x + 12} M{x - 8} {y + 16} '
        f'H{x + 8} M{x - 4} {y + 22} H{x + 4}" stroke="#212121" stroke-width="2.2" '
        f'fill="none"/>')


def box(x, y, w, h, fill, dashed=False, rx=8):
    d = ' stroke-dasharray="7 5"' if dashed else ""
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" '
        f'stroke="#333" stroke-width="1.6"{d}/>')


def header(conn, rows, x, side):
    """Draw one Arduino header column with its labels inside the board."""
    y_top = Y0[conn]
    text(x, y_top - 14, conn, "s b", "middle")
    for i, (pin, name, port, morpho) in enumerate(rows):
        y = y_top + i * PITCH
        key = USED.get((conn, pin))
        if key:
            hx, hw = (x + 10, 160) if side == "L" else (x - 180, 170)
            add(f'<rect x="{hx}" y="{y - 11}" width="{hw}" height="22" rx="3" '
                f'fill="{C[key]}" fill-opacity="0.16"/>')
        reserved = (conn, pin) in RESERVED
        fill = C[key] if key else ("#bdbdbd" if reserved else "#ffffff")
        add(f'<rect x="{x - 6}" y="{y - 6}" width="12" height="12" fill="{fill}" '
            f'stroke="#333" stroke-width="1.5"/>')
        name_cls = "t b" if key else ("t dim i" if reserved else "t dim")
        if side == "L":
            text(x + 14, y + 5, str(pin), "xs")
            text(x + 32, y + 5, name, name_cls)
            text(x + 82, y + 5, port, "t" if key else name_cls)
            text(x + 120, y + 5, morpho, "xs")
        else:
            text(x - 14, y + 5, str(pin), "xs", "end")
            text(x - 32, y + 5, name, name_cls, "end")
            text(x - 82, y + 5, port, "t" if key else name_cls, "end")
            text(x - 130, y + 5, morpho, "xs", "end")


def main():
    add(f'<?xml version="1.0" encoding="UTF-8"?>')
    add('<!-- Generated by docs/wiring/src/04-nucleo.py; edit the generator, not this file. -->')
    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" '
        f'viewBox="0 0 {WIDTH} {HEIGHT}">')
    add("""  <style>
    text { font-family: Arial, Helvetica, sans-serif; fill: #1a1a1a; }
    .title { font-size: 26px; font-weight: bold; }
    .subtitle { font-size: 15px; fill: #444; }
    .h { font-size: 17px; font-weight: bold; }
    .ph { font-size: 17px; font-weight: bold; fill: #0d47a1; }
    .t { font-size: 14px; }
    .s { font-size: 13px; fill: #333; }
    .xs { font-size: 12px; fill: #757575; }
    .dim { fill: #9e9e9e; }
    .b { font-weight: bold; }
    .i { font-style: italic; }
  </style>""")
    add(f'<rect x="0" y="0" width="{WIDTH}" height="{HEIGHT}" fill="#ffffff"/>')

    text(30, 42, "iot_robot: Nucleo-F401RE wiring", "title")
    text(30, 68, "Top view, USB end up. Pin label: header pin, Arduino name, MCU port, "
         "Morpho position (grey). Every +5V / GND flag connects to the rails on the "
         "interface perfboard.", "subtitle")

    # ---------------------------------------------------------------- wires
    # USB to the Pi
    wire("M800 130 V150", C["usb"], 4)
    # Rails feed
    wire(f"M{LEFT_X - 6} {pin_y('CN6', 5)} H300", C["v5"])
    wire(f"M{LEFT_X - 6} {pin_y('CN6', 6)} H300", C["gnd"])
    # LEFT ultrasonic. The divider order along the ECHO wire matters: Nucleo pin,
    # then the node with 2 kOhm to GND, then 1 kOhm, then the sensor's ECHO pin.
    # With the 2 kOhm on the sensor side the pin would see the full 5 V.
    wire(f"M{LEFT_X - 6} {pin_y('CN8', 1)} H390 V900", C["trig"])
    wire(f"M{LEFT_X - 6} {pin_y('CN8', 2)} H450 V900", C["echo"])
    wire(f"M450 {DIV_NODE_Y} H510", "#333", 2.4)
    wire(f"M510 {DIV_NODE_Y - 25} V900", C["gnd"])
    wire("M330 876 V900", C["v5"])
    # Battery sense
    wire(f"M{LEFT_X - 6} {pin_y('CN8', 3)} H548 V905 H760", C["sense"])
    wire("M640 905 V975", "#333", 2.4)
    wire("M700 905 V975", "#333", 2.4)
    # RIGHT ultrasonic
    wire(f"M{RIGHT_X + 6} {pin_y('CN9', 8)} H1130 V900", C["trig"])
    wire(f"M{RIGHT_X + 6} {pin_y('CN5', 1)} H1190 V900", C["echo"])
    wire(f"M1190 {DIV_NODE_Y} H1250", "#333", 2.4)
    wire(f"M1250 {DIV_NODE_Y - 25} V900", C["gnd"])
    wire("M1070 876 V900", C["v5"])
    # Servos
    wire(f"M{RIGHT_X + 6} {pin_y('CN5', 7)} H1100 V330", C["gnd"])
    wire("M1100 330 H1470 V176", C["vminus"], 4)
    wire("M1180 360 H1540 V176", C["vplus"], 4)
    wire(f"M{RIGHT_X + 6} {pin_y('CN5', 5)} H1180 V390", C["sig"])
    wire(f"M{RIGHT_X + 6} {pin_y('CN5', 4)} H1340 V390", C["sig"])
    for x in (1180, 1340):
        for dx, col in ((-6, C["sig"]), (0, C["vplus"]), (6, C["vminus"])):
            wire(f"M{x + dx} 266 V318", col, 2.2)

    # ---------------------------------------------------------------- Pi flag
    box(660, 84, 280, 46, "#e8f5e9", rx=6)
    text(800, 103, "Raspberry Pi, any USB-A port", "s b", "middle")
    text(800, 121, "USB serial /dev/stm32 (ttyACM), 115200 8N1", "s", "middle")

    # ---------------------------------------------------------------- board
    add('<rect x="560" y="160" width="480" height="712" rx="10" fill="#eceff1" '
        'stroke="#455a64" stroke-width="2"/>')
    add('<rect x="560" y="160" width="480" height="120" rx="10" fill="#cfd8dc" '
        'stroke="#455a64" stroke-width="2"/>')
    add('<line x1="560" y1="280" x2="1040" y2="280" stroke="#455a64" stroke-width="1.5" '
        'stroke-dasharray="6 4"/>')
    add('<rect x="775" y="150" width="50" height="22" rx="3" fill="#9e9e9e" stroke="#424242" '
        'stroke-width="1.5"/>')
    text(800, 192, "USB Mini-B", "s b", "middle")
    text(800, 214, "ST-LINK/V2-1: power, USB serial (USART2 PA2/PA3),", "s", "middle")
    text(800, 232, "and SWD for flashing", "s", "middle")
    text(800, 262, "(snap-off part; leave it attached)", "xs", "middle")
    text(800, 312, "NUCLEO-F401RE", "h", "middle")
    header("CN6", CN6, LEFT_X, "L")
    header("CN8", CN8, LEFT_X, "L")
    header("CN5", CN5, RIGHT_X, "R")
    header("CN9", CN9, RIGHT_X, "R")
    text(800, 820, "Grey CN7-/CN10- = same signal on the Morpho inner column.", "xs", "middle")
    text(800, 836, "Grey squares: D13 (PA5) drives LD2, the heartbeat LED;", "xs", "middle")
    text(800, 852, "D0/D1 (PA3/PA2) carry the USB serial. Leave them unwired.", "xs", "middle")

    # ---------------------------------------------------------------- rails
    box(60, 500, 240, 120, "#fffde7")
    text(75, 520, "Interface perfboard: rails", "s b")
    wire("M90 548 H300", C["v5"], 6)
    wire("M90 574 H300", C["gnd"], 6)
    text(92, 541, "+5V rail (from CN6-5)", "xs")
    text(92, 593, "GND rail (from CN6-6)", "xs")
    text(75, 611, "all +5V / GND flags tie here", "xs i")

    # ---------------------------------------------------------------- LEFT HC-SR04
    dot(450, DIV_NODE_Y)
    dot(510, DIV_NODE_Y)
    res_h(462, 498, DIV_NODE_Y)
    text(480, DIV_NODE_Y + 25, "2 kΩ", "s b", "middle")
    flag(510, DIV_NODE_Y - 25, "GND", C["gnd"])
    res_v(450, DIV_1K_Y1, DIV_1K_Y2)
    text(420, (DIV_1K_Y1 + DIV_1K_Y2) // 2 + 5, "1 kΩ", "s b", "middle")
    flag(330, 876, "+5V", C["v5"])
    box(300, 900, 240, 100, "#fff8e1")
    for x, lab in ((330, "VCC"), (390, "TRIG"), (450, "ECHO"), (510, "GND")):
        add(f'<rect x="{x - 6}" y="894" width="12" height="12" fill="#ffffff" stroke="#333" '
            'stroke-width="1.5"/>')
        text(x, 922, lab, "xs b", "middle")
    text(420, 950, "HC-SR04 LEFT", "h", "middle")
    text(420, 970, "front-left, yaw +45°", "s", "middle")
    text(420, 988, "/ultrasonic/left (D1 field)", "s", "middle")

    # ---------------------------------------------------------------- battery sense
    box(580, 880, 400, 130, "#ffffff", dashed=True)
    dot(640, 905, C["sense"])
    dot(700, 905, C["sense"])
    res_v(640, 922, 962)
    text(628, 947, "R2", "s b", "end")
    text(628, 963, "10 kΩ", "s", "end")
    add('<path d="M686 936 H714 M686 944 H714" stroke="#333" stroke-width="3"/>')
    add('<rect x="690" y="937" width="20" height="6" fill="#ffffff"/>')
    text(720, 947, "C1 100 nF", "s b")
    gnd_symbol(640, 975)
    gnd_symbol(700, 975)
    box(760, 892, 210, 36, "#fafafa", rx=4)
    text(865, 907, "from R1 100 kΩ at J1", "s b", "middle")
    text(865, 922, "(+24 V switched, power diagram)", "xs", "middle")
    text(760, 962, "Battery sense (optional)", "s b")
    text(760, 980, "V(PA4) = V(battery) / 11", "s")
    text(760, 998, "ground symbols = GND rail", "xs i")

    # ---------------------------------------------------------------- RIGHT HC-SR04
    dot(1190, DIV_NODE_Y)
    dot(1250, DIV_NODE_Y)
    res_h(1202, 1238, DIV_NODE_Y)
    text(1220, DIV_NODE_Y + 25, "2 kΩ", "s b", "middle")
    flag(1250, DIV_NODE_Y - 25, "GND", C["gnd"])
    res_v(1190, DIV_1K_Y1, DIV_1K_Y2)
    text(1160, (DIV_1K_Y1 + DIV_1K_Y2) // 2 + 5, "1 kΩ", "s b", "middle")
    flag(1070, 876, "+5V", C["v5"])
    box(1040, 900, 240, 100, "#fff8e1")
    for x, lab in ((1070, "VCC"), (1130, "TRIG"), (1190, "ECHO"), (1250, "GND")):
        add(f'<rect x="{x - 6}" y="894" width="12" height="12" fill="#ffffff" stroke="#333" '
            'stroke-width="1.5"/>')
        text(x, 922, lab, "xs b", "middle")
    text(1160, 950, "HC-SR04 RIGHT", "h", "middle")
    text(1160, 970, "front-right, yaw -45°", "s", "middle")
    text(1160, 988, "/ultrasonic/right (D2 field)", "s", "middle")

    # ---------------------------------------------------------------- servos + BEC
    box(1430, 92, 150, 84, "#ffffff", dashed=True)
    text(1505, 114, "Servo BEC (opt.)", "s b", "middle")
    text(1505, 132, "in: 24 V motor bus", "xs", "middle")
    text(1505, 148, "out: 5-6 V, 3 A", "xs", "middle")
    text(1462, 170, "-", "s b", "middle")
    text(1532, 170, "+", "s b", "middle")
    for x, name, ch in ((1180, "YAW servo", "PA6 TIM3_CH1"), (1340, "PITCH servo", "PA7 TIM3_CH2")):
        box(x - 70, 196, 140, 70, "#ffffff", dashed=True)
        text(x, 216, name, "s b", "middle")
        text(x, 234, ch, "xs", "middle")
        text(x, 250, "50 Hz, 1500 µs centre", "xs", "middle")
        add(f'<rect x="{x - 10}" y="318" width="20" height="84" rx="3" fill="#424242"/>')
        for y, col in ((330, C["vminus"]), (360, C["vplus"]), (390, C["sig"])):
            add(f'<circle cx="{x}" cy="{y}" r="5" fill="{col}" stroke="#ffffff" '
                'stroke-width="1.5"/>')
    text(1356, 422, "servo plug, top to bottom:", "xs")
    text(1356, 438, "- brown, + red, S orange", "xs")
    dot(1100, 330, C["gnd"])

    # ---------------------------------------------------------------- notes
    add('<rect x="30" y="640" width="250" height="370" rx="8" fill="#fafafa" stroke="#999" '
        'stroke-width="1.2"/>')
    text(45, 666, "Build notes", "ph")
    notes_left = [
        "Mount the rails, both",
        "dividers and R2/C1 on one",
        "small perfboard next to",
        "the Nucleo.",
        "",
        "ECHO swings 0-5 V. Divider",
        "order: Nucleo pin, node",
        "with 2 kΩ to GND, 1 kΩ,",
        "ECHO. The pin sees 3.33 V.",
        "These pins are 5 V tolerant",
        "in digital mode; the divider",
        "keeps margin anyway.",
        "",
        "TRIG needs no shifter:",
        "3.3 V is a valid TTL high.",
        "",
        "ST-LINK USB budget: about",
        "300 mA incl. the 5V pin.",
    ]
    for i, line in enumerate(notes_left):
        text(45, 692 + i * 17, line, "s")

    add('<rect x="1300" y="520" width="280" height="490" rx="8" fill="#fafafa" stroke="#999" '
        'stroke-width="1.2"/>')
    text(1315, 546, "Servos (optional)", "ph")
    notes_right = [
        "Gizmo actuator is TBD; this",
        "assumes two hobby servos.",
        "",
        "Servo power comes from the",
        "BEC only. Never from the",
        "Nucleo 5V pin: a stalled",
        "servo draws amps.",
        "",
        "Join BEC - to Nucleo GND",
        "(CN5-7): the PWM signal",
        "needs a common ground.",
        "",
        "PWM: 50 Hz, 1500 µs = 0°,",
        "±1000 µs = ±90°, firmware",
        "clamps 500-2500 µs.",
        "",
        "Ultrasonics",
        "",
        "Range 2 cm - 4 m, 15° cone.",
        "Aim the two sensors so the",
        "cones do not overlap.",
        "Allow 60 ms or more",
        "between pings.",
    ]
    for i, line in enumerate(notes_right):
        cls = "ph" if line == "Ultrasonics" else "s"
        text(1315, 572 + i * 17, line, cls)

    # ---------------------------------------------------------------- legend
    lx, ly = 30, 1050
    text(lx, ly + 5, "Wires:", "s b")
    items = [("v5", "+5V"), ("gnd", "GND"), ("trig", "TRIG"), ("echo", "ECHO"),
             ("sense", "battery sense"), ("sig", "servo signal"), ("vminus", "servo -"),
             ("usb", "USB")]
    x = lx + 60
    for key, label in items:
        wire(f"M{x} {ly} H{x + 36}", C[key], 4)
        text(x + 44, ly + 5, label, "s")
        x += 44 + 12 + len(label) * 7.5 + 30
    text(x + 10, ly + 5, "Dashed boxes: optional parts. Pin tables: docs/wiring.md.", "s i")

    add("</svg>")
    OUT.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
