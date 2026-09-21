# iot_robot wiring

How to connect the iot_robot hardware: battery and power distribution, the two CAN
wheel motors, the Nucleo sensor board, and the camera. Each section has a diagram
and a pin-by-pin table with the same connections. Where the two disagree, the table
is the reference; please report the mismatch.

The diagrams are SVG files in [`docs/wiring/`](wiring/). Open one in its own browser
tab to zoom in, or print it (landscape, fit to page).

> **Status.** Drawn from manufacturer datasheets and manuals (see [Sources](#sources)),
> not yet checked against the assembled robot. Items marked **TBD** depend on parts
> that are not confirmed yet; they are collected under [Open items](#open-items-tbd).
> Where an assumption changes the wiring, the section says what to do instead.

## Overview

![iot_robot system overview](wiring/01-system-overview.svg)

- **Power.** One 6S LiPo powers everything. After the main fuse F1 and the main
  switch SW1, the switched +24 V (junction **J1**) splits two ways:
  - through the E-stop S1 to the **motor bus** (junction **J2**): both AK45-10
    drives and the optional servo BEC;
  - through fuse F2 to a **buck converter** that makes 5.1 V for the Raspberry Pi.

  The E-stop cuts the motor bus only. The Pi, the camera and the Nucleo stay on.
- **Data.** The Raspberry Pi talks to:
  - both motors over CAN at 1 Mbit/s, through a USB-CAN adapter (`can0`);
  - the Nucleo-F401RE over the ST-LINK USB virtual COM port (115200 8N1). It shows up
    as `/dev/ttyACM<n>`; the udev rule in `deploy/udev/99-iot-robot-stm32.rules` adds
    the stable name `/dev/stm32`;
  - the Camera Module v2.1 over the CSI ribbon.
- **Nucleo.** Reads two HC-SR04 ultrasonic sensors. It can also read the battery
  voltage and drive two gizmo servos; both are optional.
- **Ground.** All grounds are common. Battery negative is the single 0 V reference.
  The star point is the **0 V terminal block** (or bus bar): battery negative, both
  motor returns, the buck IN-, the servo BEC IN- and the CAN adapter GND each land on
  it with their own wire. Do not stack the motor returns on the buck's IN- screw
  terminal; that would push motor current through the buck's small terminal.

| Diagram | Content |
|---|---|
| [01-system-overview.svg](wiring/01-system-overview.svg) | Every part and the links between them |
| [02-power-distribution.svg](wiring/02-power-distribution.svg) | Battery, fuses, switch, E-stop, motor bus, buck, battery sense, relay option |
| [03-can-bus.svg](wiring/03-can-bus.svg) | USB-CAN adapter to both motors, termination, CAN IDs |
| [03-can-bus-harness.svg](wiring/03-can-bus-harness.svg) | The same CAN bus wire by wire (WireViz, wide; open separately) |
| [04-nucleo.svg](wiring/04-nucleo.svg) | Nucleo header pins to the ultrasonics, servos and battery sense |
| [05-camera-ribbon.svg](wiring/05-camera-ribbon.svg) | Camera ribbon and its orientation at both ends |

## Bill of materials

### Parts you have

| Part | Qty | Notes |
|---|---|---|
| CubeMars AK45-10 KV75 actuator | 2 | Left wheel CAN ID 1, right wheel CAN ID 2. Version **TBD**: the original has separate XT30 power and 4-pin CAN connectors (drawn here). V3.0 has one XT30(2+2) plug, see [V3.0 variant](#ak45-10-v30-variant). Each motor ships with a 16 AWG XT30 power lead and CAN plugs. |
| USB-CAN adapter ("usb2can") | 1 | Model **TBD**. It needs a Linux SocketCAN driver: gs_usb/candleLight, or slcan. It should also have a switchable 120 Ω terminator. |
| Raspberry Pi | 1 | Model **TBD**. The diagrams assume a Pi 5 and note where a Pi 4 differs. |
| Raspberry Pi Camera Module v2.1 (Sony IMX219) | 1 | Ships with a 15-pin Standard-Standard ribbon, which fits a Pi 4 or older only. |
| STM32 Nucleo-F401RE | 1 | Plus a USB-A to Mini-B cable. |
| HC-SR04 ultrasonic sensor | 2 | 5 V modules, pins VCC / TRIG / ECHO / GND. |
| 6S LiPo battery | 1 | 22.2 V nominal, 25.2 V full. Capacity and plug **TBD**. |
| 24 V to 5 V buck converter | 1 | Rating **TBD**, see [Buck converter](#feeding-51-v-into-the-pi). |
| Pan/tilt gizmo | 1 | Actuator **TBD**. The wiring assumes two hobby servos. |

### Parts you'll also need

| Part | Qty | Specification | Used for |
|---|---|---|---|
| Inline blade fuse holder | 2, up to 4 | ATO/ATC, 32 V DC. F1 with 16 AWG leads; F2 and F3 with 20 AWG; F4 with 22 AWG | F1, F2; F3 only with the servo BEC; F4 only with relay K1 |
| Blade fuse 15 A | 1 + spares | ATO/ATC, 32 V DC | F1, main fuse at the battery |
| Blade fuse 3 A | 1-2 + spares | ATO/ATC, 32 V DC | F2 (buck branch, at J1); F3 (servo BEC branch, at J2, optional) |
| Blade fuse 1 A | 1 + spares | ATO/ATC, 32 V DC | F4 (relay coil loop, at J1), only with option C |
| Main switch SW1 | 1 | 20 A DC or more at 30 V DC or more. It must have a **DC** rating; an AC rating alone does not count. | Turns the whole robot off |
| E-stop S1 | 1 | Latching mushroom head, NC contact. If the contact is rated 10 A DC or more at 24 V, it switches the motor bus directly. Otherwise use it with K1. | Cuts the motor bus |
| Relay K1 (only if S1 is rated below 10 A DC) | 1 | 24 V coil, contacts 20 A DC or more, e.g. a 24 V automotive relay rated at 28 V DC | Switches the motor bus for S1 |
| Diode D1 (with K1) | 1 | 1N4007 | Flyback diode across the K1 coil |
| Battery connector | 1 | Mate for the battery plug (**TBD**, often XT60) | Battery lead |
| AMASS XT30U-F / XT30UPB-F plug | 2 | Mates the drive's XT30PW-M socket | Motor power leads (one pair is supplied with each motor) |
| CJT A1257H-4P housing + A1257-TP crimp terminals | 4 | 1.25 mm pitch, 4-pin. The terminals take AWG 28-32 wire with insulation 1.0 mm OD or less [[a1257]], and need a fine-pitch crimp tool. | CAN plugs (two are supplied with each motor) |
| Terminal blocks or lever connectors | 3 | 20 A or more | J1, J2 and the 0 V bus (the star point) |
| 120 Ω resistor | 1 | 1/4 W | CAN terminator at the right motor |
| 1 kΩ resistor | 2 | 1/4 W | ECHO dividers |
| 2 kΩ resistor | 2 | 1/4 W (2.2 kΩ also works: 3.44 V) | ECHO dividers |
| 100 kΩ and 10 kΩ resistor | 1 each | 1/4 W, 1 % metal film | Battery sense (optional) |
| 100 nF ceramic capacitor | 1 | 50 V | Battery sense filter (optional) |
| Perfboard | 1 | About 5 x 7 cm | Interface board: +5V/GND rails, dividers, R2/C1 |
| Dupont jumpers and housings | about 15 | Male pins fit the Nucleo's Arduino sockets; female fit the Morpho pins | Nucleo signals |
| USB-C power lead or USB-C pigtail | 1 | 18 AWG or heavier, 30 cm or shorter | Buck output to the Pi (option A1) |
| Standard-Mini camera cable | 1 | Raspberry Pi part, 300 or 500 mm | Pi 5 only |
| Servo BEC (optional) | 1 | Input covers 6S (26 V or more), output 5-6 V, 3 A | Gizmo servos |
| LiPo voltage alarm (recommended) | 1 | Plugs into the balance lead | Warns before 3.5 V per cell |
| Heat-shrink, ferrules, crimp tool, multimeter | - | - | Assembly and checks |

### Wire

| Run | Gauge | Colour |
|---|---|---|
| Battery, F1, SW1, J1, S1, J2, both motors, and the 0 V bus | 16 AWG silicone | red (+) / black (0 V) |
| F2 to buck input; F3 to servo BEC input | 20 AWG | red / black |
| Relay coil loop (option C, after F4) | 22 AWG | red / black |
| Buck output to the Pi | 18 AWG, 30 cm or shorter | red / black |
| CAN_H / CAN_L | 28 AWG **stranded**, twisted pair, insulation 1.0 mm OD or less (e.g. PTFE hook-up wire) | white / blue |
| CAN GND reference | 22 AWG | black |
| Nucleo rails, sensor, servo and battery sense signals | 22-26 AWG | as in the Nucleo diagram |

- **Branch fuses sit at the source end of every thinner branch.** A short in a 20 or
  22 AWG lead can draw 10-15 A without blowing F1, and those wires cannot carry that for
  long. F2 and F4 mount directly at J1, F3 directly at J2, so no unfused thin wire
  leaves a junction.
- **The CAN wire gauge is set by the plug.** Every CAN wire ends in an A1257H-4P plug,
  and its crimp terminals take only AWG 28-32 [[a1257]]. The CAN lead supplied with the
  motors is 30 AWG [[akdrv]]. Do not use solid-core wire such as Cat5: it does not crimp
  reliably and breaks under vibration. If you want heavier wire for a longer run, keep
  a short pigtail of the supplied 30 AWG lead on each plug and splice it to the heavier
  stranded pair away from the connector.

## 1. Power distribution

![Power distribution](wiring/02-power-distribution.svg)

- **F1 sits within 10 cm of battery +.** It protects the wiring, so nothing may
  connect between the battery and F1.
- **SW1 turns everything off.** **S1 only cuts the motor bus.** Pressing the E-stop
  removes motor power, while the Pi keeps running, logging and reachable.
- **The E-stop is a coast stop, not a brake.** An unpowered drive produces no torque,
  so the wheels roll out. The 10:1 planetary gearbox is backdrivable, so on a slope the
  robot can keep rolling.
- The buck converter and the battery-sense tap connect at J1: after SW1 (so they
  draw nothing when the robot is off) and before S1 (so the E-stop does not cut
  them).
- The E-stop cuts only the + side. The 0 V bus stays connected, so CAN and sensor
  grounds keep their shared reference.

> **Warning: after an E-stop press the robot software must be restarted, and releasing
> the web E-stop afterwards can resume motion.** ROS cannot see the hardware E-stop
> directly. The robot uses the `cubemars_hardware_safe` plugin: when the drives lose
> power their status frames stop, and after 100 ms the plugin stops sending commands and
> latches that stop (see [Stopping the wheels](hardware.md#stopping-the-wheels)).
> Releasing S1 therefore does not move the wheels; the drives boot and receive nothing.
> Restarting the robot software clears the latch. `robot.launch.py` starts the web page
> with its E-stop engaged (`start_estopped:=true`, see
> [web.md](web.md#safety-and-security)), so the restarted robot stays still until
> someone releases the stop on the page. From then on, whatever is still commanding
> motion (the follower, teleop) drives the robot again. Recover in this order:
>
> 1. Stop every node that commands motion: close teleop, stop the follower, and release
>    the drive controls on the web page.
> 2. Release S1. Wait until both drives' blue power LEDs are on [[akdrv]], then
>    give them another second or two to boot.
> 3. Restart the robot software: Ctrl-C and start again, or
>    `sudo systemctl restart iot-robot`.
> 4. Release the E-stop on the web page, then start the follower or teleop again, only
>    when the robot may move.
>
> Possible improvement, not built: a second contact block on S1, read by a Nucleo input
> and reported by `stm32_bridge`, could latch `/e_stop` automatically. That needs
> firmware and driver changes as well as wiring.

### Power harness

| From | Pin | To | Pin | Wire | Notes |
|---|---|---|---|---|---|
| Battery | + | F1 15 A | in | 16 AWG red | F1 within 10 cm of battery +. Battery plug **TBD**. |
| F1 | out | SW1 | in | 16 AWG red | |
| SW1 | out | J1 | - | 16 AWG red | J1: switched +24 V junction (terminal block) |
| J1 | - | S1 E-stop | NC contact, first terminal | 16 AWG red | NC contact terminals are usually numbered 11-12 or 21-22. Wiring with K1: see [option C](#e-stop-through-a-relay-option-c). |
| S1 E-stop | NC contact, second terminal | J2 | - | 16 AWG red | J2: +24 V motor bus junction |
| J2 | - | AK45-10 LEFT | XT30PW-M pin 2 (+) | 16 AWG red | Use the supplied XT30 lead. Check polarity against the housing markings. |
| J2 | - | AK45-10 RIGHT | XT30PW-M pin 2 (+) | 16 AWG red | As above |
| J2 | - | F3 3 A | in | - | Optional, with the servo BEC. Fuse holder mounted directly at J2. |
| F3 3 A | out | Servo BEC | IN+ | 20 AWG red | Optional |
| J1 | - | F2 3 A | in | - | Fuse holder mounted directly at J1, so the 20 AWG lead is fused from its first centimetre |
| F2 | out | Buck converter | IN+ | 20 AWG red | |
| Battery | - | 0 V bus | - | 16 AWG black | Terminal block or bus bar: the ground star point |
| 0 V bus | - | AK45-10 LEFT | XT30PW-M pin 1 (-) | 16 AWG black | |
| 0 V bus | - | AK45-10 RIGHT | XT30PW-M pin 1 (-) | 16 AWG black | |
| 0 V bus | - | Buck converter | IN- | 20 AWG black | Own wire from the block; the motor returns do not pass through the buck |
| 0 V bus | - | Servo BEC | IN- | 20 AWG black | Optional |
| 0 V bus | - | USB-CAN adapter | GND | 22 AWG black | See the [CAN harness](#can-harness) |
| Buck converter | OUT+ | Raspberry Pi | USB-C VBUS (A1) or GPIO pins 2 and 4 (A2) | 18 AWG red | 30 cm or shorter. See [Feeding 5.1 V into the Pi](#feeding-51-v-into-the-pi). |
| Buck converter | OUT- | Raspberry Pi | USB-C GND (A1) or GPIO pins 6 and 14 (A2) | 18 AWG black | |
| J1 | - | R1 100 kΩ | lead 1 | - | Optional battery sense. R1 is soldered at J1, see below. |
| R1 100 kΩ | lead 2 | Nucleo | CN8-3 (A2, PA4) | 22-26 AWG purple | R2 and C1 sit at the Nucleo end, see the [Nucleo table](#nucleo-connections) |

The XT30PW-M pin numbers (1 = -, 2 = +) are from the CubeMars AK driver manual
V1.0.18, section 1.2.3 [[akdrv]]. One CubeMars installation guide has a power
colour table that looks swapped [[ak40]]. Trust the "+" / "-" markings on the housing
and a multimeter, not the wire colour.

### Current budget and fuse sizing

| Load | Current from the battery | Source |
|---|---|---|
| AK45-10, each | 2.1 A rated, 5 A peak | [[ak45]] |
| Buck for a Pi 5 (5.1 V x 5 A at about 90 % efficiency, 22 V in) | about 1.3 A | calculated |
| Servo BEC (6 V x 3 A) | about 0.9 A | calculated |
| **Total peak** | **about 12 A** | |

F1 at 15 A clears the combined peak with margin, and 16 AWG silicone wire carries
that over short runs. F2 at 3 A covers the buck's worst case plus the inrush of its
input capacitors. F3 at 3 A does the same for the servo BEC, and F4 at 1 A (relay
option only) covers a relay coil, which draws well under 0.5 A at 24 V. The drive board accepts 18-28 V (16-28 V on the mini driver board)
[[akdrv]], so a 6S pack between 21 V and 25.2 V is within range.

Stop driving at about 21 V (3.5 V per cell). Below that, both the pack life and the
drives' input margin suffer. The optional battery sense or a balance-lead alarm
tells you when to stop.

### Feeding 5.1 V into the Pi

The Pi needs 5.1 V. It logs an undervoltage warning below 4.63 V [[rpipower]]. Set
the buck output to 5.1 V with no load connected, then check it again under load at
the Pi end of the lead.

**Buck converter rating** (model **TBD**):

| Pi model | Buck output | Why |
|---|---|---|
| Raspberry Pi 5 | 5.1 V, 5 A | The official Pi 5 PSU is 27 W (5.1 V, 5 A) [[rpipower]] |
| Raspberry Pi 4 | 5.1 V, 3 A | The official Pi 4 PSU is 3 A [[rpipower]] |

For either model, the input must be rated 30 V or more. A full 6S pack is 25.2 V,
and motor braking can push the bus a little higher.

**Option A1 (recommended): USB-C socket.**
- Run a short, thick USB-C power lead from the buck output (a buck with a USB-C
  output, or a USB-C pigtail).
- This keeps the Pi's own input protection in the circuit.
- A plain buck does not negotiate USB-PD. A Pi 5 then assumes a 3 A supply and limits
  its USB ports to 600 mA in total (1.6 A with a 5 A supply) [[rpipower]].
- 600 mA is enough for the USB-CAN adapter and the Nucleo together. They draw an
  estimated 0.2-0.3 A.
- If the buck really delivers 5 A, you can lift the limit and silence the low-power
  warning. Set `PSU_MAX_CURRENT` in the Pi 5 bootloader EEPROM [[rpieeprom]]:

  ```bash
  sudo rpi-eeprom-config --edit
  # add this line, save, and reboot:
  PSU_MAX_CURRENT=5000
  ```

**Option A2: GPIO header pins.**
- Connect 5.1 V to pins 2 and 4 and 0 V to pins 6 and 14.
- This bypasses the Pi's input protection [[bret]]: a failed buck can put up to 25 V
  straight onto the Pi. Use it only with a buck that has output over-voltage
  protection.
- One Dupont contact carries only about 1-1.5 A, so even both 5 V pins together are
  good for about 3 A. That suits a Pi 4, not a fully loaded Pi 5.
- Use crimped housings, not loose jumpers: a **1x3 housing on pins 2, 4 and 6**
  (5 V, 5 V, GND) plus a single contact on pin 14 (GND). Pins 2, 4 and 6 are
  neighbours in the even row, and pin 14 is four positions further along. Do not use a
  2x2 housing: it also covers odd-row pins (1/3 = 3V3/GPIO2 or 3/5 = GPIO2/GPIO3), and
  5.1 V or 0 V landing there can short the 3V3 rail or drive a GPIO.
- On a Pi 5 the 600 mA USB limit still applies unless you set `PSU_MAX_CURRENT` (above)
  or `usb_max_current_enable=1` in `/boot/firmware/config.txt` [[rpiconfig]].

A Raspberry Pi 4 has no `PSU_MAX_CURRENT` setting. Its USB ports get 1.2 A with any
supply [[rpipower]].

### Battery sense (optional)

A 100 kΩ / 10 kΩ divider scales the battery voltage for the Nucleo ADC on PA4:

- V(PA4) = V(battery) x 10 / (100 + 10) = V(battery) / 11
- Full pack, 25.2 V: PA4 sees 2.29 V. The ADC full scale of 3.3 V corresponds to 36.3 V.
- **R1 (100 kΩ) sits at the J1 end.** If the long signal wire chafes through to 0 V,
  only 0.25 mA flows and F1 does not blow.
- **R2 (10 kΩ) and C1 (100 nF) sit at the Nucleo.** C1 gives the ADC a low-impedance
  source for its sampling capacitor and filters motor noise.
- The tap is after SW1, so it draws nothing while the robot is off.
- Battery sensing is off by default in the firmware. The Nucleo then sends no `V:`
  field.

### E-stop through a relay (option C)

Use this if S1's contact is rated below 10 A DC. Many panel E-stops list a high AC
rating and a much lower DC rating, so check the DC figure. S1 then switches only the
relay coil current.

| From | Pin | To | Pin | Wire | Notes |
|---|---|---|---|---|---|
| J1 | - | K1 | contact common (30 on an automotive relay) | 16 AWG red | |
| K1 | NO contact (87) | J2 | - | 16 AWG red | Replaces the S1-to-J2 link |
| J1 | - | F4 1 A | in | - | Fuse holder mounted directly at J1 |
| F4 1 A | out | S1 E-stop | NC contact, first terminal | 22 AWG red | Coil current only |
| S1 E-stop | NC contact, second terminal | K1 | coil + (86) | 22 AWG red | |
| K1 | coil - (85) | 0 V bus | - | 22 AWG black | |
| D1 1N4007 | cathode (stripe) | K1 | coil + (86) | - | Solder across the coil terminals |
| D1 1N4007 | anode | K1 | coil - (85) | - | |

Pressing S1 releases K1 and the motor bus goes dead, while the Pi and the Nucleo stay
on. The wheels coast. Releasing S1 powers the drives up again; they reboot and wait
for commands, which only come once the robot software is restarted. Follow the
recovery order in the [warning above](#1-power-distribution).

With either option, closing the motor bus charges the drives' input capacitors. Expect
a small spark at the XT30 or at the contact.

## 2. CAN bus

![CAN bus](wiring/03-can-bus.svg)

The bus is one line: adapter, LEFT motor, RIGHT motor, terminator. It has a 120 Ω
terminator at each end. The wire-by-wire view is
[03-can-bus-harness.svg](wiring/03-can-bus-harness.svg). It is generated with WireViz
and is wide, so open it in its own tab.

The original AK45-10 CAN port is a CJT A1257WR-S-4P [[akdrv]]:

| AK45-10 CAN pin | Signal |
|---|---|
| 1 | CAN_L |
| 2 | CAN_H |
| 3 | CAN_H (bridged to pin 2 inside the drive) |
| 4 | CAN_L (bridged to pin 1 inside the drive) |

Because the pairs are bridged, the bus passes through each motor. Use the same rule
on both motors: **bus in on pins 1/2, bus out on pins 3/4**.

### CAN harness

| From | Pin | To | Pin | Wire | Notes |
|---|---|---|---|---|---|
| Raspberry Pi | any USB-A port | USB-CAN adapter | USB | USB cable | gs_usb adapters appear as `can0` |
| USB-CAN adapter | CAN_H | AK45-10 LEFT | 2 (CAN_H) | 28 AWG stranded white, twisted with CAN_L | Bus in. Adapter's 120 Ω terminator **ON** (jumper or DIP switch) |
| USB-CAN adapter | CAN_L | AK45-10 LEFT | 1 (CAN_L) | 28 AWG stranded blue | |
| AK45-10 LEFT | 3 (CAN_H) | AK45-10 RIGHT | 2 (CAN_H) | 28 AWG stranded white, twisted with CAN_L | Out on 3, in on 2: the pair crosses over. This is intended. |
| AK45-10 LEFT | 4 (CAN_L) | AK45-10 RIGHT | 1 (CAN_L) | 28 AWG stranded blue | |
| AK45-10 RIGHT | 3 (CAN_H) | R_T 120 Ω | lead 1 | 28 AWG white, under 5 cm | Bus end B. The resistor's own leads are too thick for the A1257 terminal: crimp short 28 AWG leads into the plug, solder R_T to them and heat-shrink over it. |
| AK45-10 RIGHT | 4 (CAN_L) | R_T 120 Ω | lead 2 | 28 AWG blue, under 5 cm | |
| USB-CAN adapter | GND | 0 V bus (terminal block) | - | 22 AWG black | The star point, on its own terminal. Required if the adapter is galvanically isolated. |

Match wires by pin number, not by colour. Colours of supplied leads vary between
manual versions. The CubeMars CAN lead in the accessory kit may populate only one
pair of its 4-pin plug. Check which pins it uses with a multimeter before building
with it.

If your adapter has a DB9 connector (for example Inno-maker USB2CAN), the pinout
follows CiA 303-1 [[inno]]:

| DB9 pin | Signal |
|---|---|
| 2 | CAN_L |
| 3 | GND |
| 7 | CAN_H |

### Termination check

Do this with everything powered **off** and plugged together. Measure the resistance
between CAN_H and CAN_L at any connector [[kvaser]]:

| Reading | Meaning |
|---|---|
| about 60 Ω | Correct: two 120 Ω terminators in parallel |
| about 120 Ω | One terminator missing (adapter switch off, or R_T not fitted) |
| about 40 Ω | Three terminators |
| about 30 Ω | Four terminators: both drives have built-in ones, see below |
| open | No terminator, or a broken wire |
| near 0 Ω | CAN_H shorted to CAN_L |

The CubeMars manual does not say whether the drive has a built-in terminator. Before
building the harness, measure between pins 2 and 1 on one bare motor:

- **Open:** no built-in terminator. Wire the bus as drawn.
- **About 120 Ω:** the drive has one, and since both drives are the same, so does the
  other. Leave out R_T **and** switch the adapter's terminator off. The two drives then
  form the two terminators, and the bus reads about 60 Ω again; check it. The LEFT
  drive's terminator then sits mid-bus and the adapter end is unterminated. That is
  acceptable for a bus under 1 m like this one. For a longer bus, rearrange it so the
  two drives are at the physical ends, with the adapter on a short stub (under 30 cm)
  between them.

### Setting the CAN IDs

Every AK motor ships with CAN ID 1, so two new motors on one bus both answer to ID 1.
Set the IDs before wiring the bus, one motor at a time:

1. Connect one motor to a CubeMars R-Link. Use its UART port (3-pin: 1 GND, 2 TX,
   3 RX) [[akdrv]]. Open the CubeMars upper-computer tool.
2. Set the ID: **LEFT = 1, RIGHT = 2**.
3. Check that the CAN rate is 1 Mbit/s and enable "Send status over CAN".
   `cubemars_hardware` relies on the status frames [[cmhw]].
4. Label the motor with its ID, then do the other one.

These IDs are the defaults of the `left_can_id` / `right_can_id` launch arguments in
`iot_robot_bringup/launch/robot.launch.py`. If you choose other IDs, pass them there.
Direction is also a launch setting (`left_direction`, `right_direction`), not a wiring
change: the drive commutates internally, so the motor leads cannot be swapped to
reverse it.

### Bringing up can0

On the robot, `deploy/network/80-iot-robot-can0.network` configures `can0` at 1 Mbit/s
through systemd-networkd whenever a gs_usb adapter appears. By hand, the command is
the one from the `cubemars_hardware` README [[cmhw]]. The Pi has no system ROS
install: ROS commands run inside the pixi environment `robot`, so they need the
`pixi run -e robot` prefix (see [hardware.md](hardware.md#ros-environment)).

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0      # expect "bitrate 1000000", state ERROR-ACTIVE
# both motors (IDs 1 and 2) reporting:
pixi run -e robot ros2 run iot_robot_drivers cubemars_tool check
# decoded status frames:
pixi run -e robot ros2 run iot_robot_drivers cubemars_tool monitor
```

slcan adapters do not take their bitrate from `ip link`. Use `deploy/scripts/can_up.sh`
for those.

### Rules for this bus

- One chain, no star: adapter, LEFT, RIGHT, terminator.
- Exactly two 120 Ω terminators, one at each physical end.
- Twist CAN_H and CAN_L together along the whole length. Route the pair away from the
  XT30 power leads.
- Keep any branch (stub) off the chain under 30 cm.
- The motor CAN port has no GND pin. The drives reference CAN to their XT30 power
  negative. The adapter's GND wire ties the adapter to the same 0 V, at the 0 V
  terminal block.
- Every wire into an A1257H-4P plug is 28 AWG stranded, or the supplied 30 AWG lead
  (see [Wire](#wire)).
- Plug or unplug CAN only with the motor bus off.

### AK45-10 V3.0 variant

AK45-10 V3.0 motors combine power and CAN in one XT30(2+2) socket per motor
[[akv3]], [[ak45v3]]:

| XT30(2+2) pin | Signal |
|---|---|
| 1 | + (24 V) |
| 2 | - (0 V) |
| 3 | CAN_H |
| 4 | CAN_L |

Each V3.0 motor ships with a 100 mm pigtail: an XT30(2+2)-F plug at one end, stripped
and tinned wires at the other. With only one CAN pair per motor, the bus cannot pass
through the drive:

- Run the twisted pair from the adapter to a splice at the LEFT motor, then on to a
  splice at the RIGHT motor. End it with R_T there.
- Solder each pigtail's CAN_H/CAN_L into its splice. That keeps each stub about
  10 cm, well under 30 cm.
- The power pins go to J2 and the 0 V bus exactly as in the power table.

## 3. Nucleo-F401RE sensor board

![Nucleo wiring](wiring/04-nucleo.svg)

The Nucleo is powered and talks to the Pi over one USB cable: the ST-LINK Mini-B port
to any Pi USB-A port. Its USART2 (PA2/PA3) goes to the ST-LINK virtual COM port, and
the same port is used for flashing over SWD. Leave the ST-LINK part attached, and
leave jumper JP5 in its default U5V position (power from the ST-LINK USB) [[um1724]].

The interface perfboard carries a +5V rail and a GND rail, both fed from the Nucleo
header. It also holds the two ECHO dividers and R2/C1 of the battery sense. In the
diagram, every +5V or GND flag means "connect to that rail".

### Nucleo connections

Nucleo pins are given as `header-pin (Arduino name, MCU port)`. The Morpho column is
the same signal on the long male header (CN7/CN10), if you prefer female Dupont
leads there. Pin data: UM1724 Rev 17, Figure 17 and Tables 16 and 29 [[um1724]].

| From | Pin | To | Pin | Wire | Notes |
|---|---|---|---|---|---|
| Raspberry Pi | any USB-A port | Nucleo ST-LINK | USB Mini-B | USB cable | Power + serial, `/dev/stm32`, 115200 8N1 |
| Nucleo | CN6-5 (+5V; Morpho CN7-18) | Perfboard | +5V rail | 22 AWG red | Sensor supply. Do not use CN10-8 (U5V). |
| Nucleo | CN6-6 (GND; Morpho CN7-20) | Perfboard | GND rail | 22 AWG black | |
| Perfboard | +5V rail | HC-SR04 LEFT | VCC | 24 AWG red | |
| Perfboard | GND rail | HC-SR04 LEFT | GND | 24 AWG black | |
| Nucleo | CN8-1 (A0, PA0; Morpho CN7-28) | HC-SR04 LEFT | TRIG | 24 AWG yellow | Direct: 3.3 V is a valid TTL high |
| HC-SR04 LEFT | ECHO | 1 kΩ (perfboard) | lead 1 | 24 AWG green | 0-5 V pulse |
| 1 kΩ (perfboard) | lead 2 = divider node | Nucleo | CN8-2 (A1, PA1; Morpho CN7-30) | 24 AWG green | 3.33 V high |
| Divider node LEFT | - | 2 kΩ | to GND rail | on perfboard | |
| Perfboard | +5V rail | HC-SR04 RIGHT | VCC | 24 AWG red | |
| Perfboard | GND rail | HC-SR04 RIGHT | GND | 24 AWG black | |
| Nucleo | CN9-8 (D7, PA8; Morpho CN10-23) | HC-SR04 RIGHT | TRIG | 24 AWG yellow | |
| HC-SR04 RIGHT | ECHO | 1 kΩ (perfboard) | lead 1 | 24 AWG green | |
| 1 kΩ (perfboard) | lead 2 = divider node | Nucleo | CN5-1 (D8, PA9; Morpho CN10-21) | 24 AWG green | |
| Divider node RIGHT | - | 2 kΩ | to GND rail | on perfboard | |
| R1 100 kΩ (at J1) | lead 2 | Nucleo | CN8-3 (A2, PA4; Morpho CN7-32) | 22-26 AWG purple | Optional battery sense, ADC1_IN4 |
| CN8-3 node | - | R2 10 kΩ | to GND rail | on perfboard | Optional |
| CN8-3 node | - | C1 100 nF | to GND rail | on perfboard | Optional |
| Nucleo | CN5-5 (D12, PA6; Morpho CN10-13) | YAW servo | S (signal, orange) | 24 AWG orange | Optional. TIM3_CH1, 50 Hz |
| Nucleo | CN5-4 (D11, PA7; Morpho CN10-15) | PITCH servo | S (signal, orange) | 24 AWG orange | Optional. TIM3_CH2, 50 Hz |
| Servo BEC | OUT+ | YAW and PITCH servos | + (red) | 22 AWG red | Optional. Never from the Nucleo 5V pin. |
| Servo BEC | OUT- | YAW and PITCH servos | - (brown) | 22 AWG brown | Optional |
| Servo BEC | OUT- | Nucleo | CN5-7 (GND; Morpho CN10-9) | 22 AWG black | Common ground for the PWM signal |
| Servo BEC | IN+ / IN- | J2 (through F3 3 A) / 0 V bus | - | 20 AWG | See the power table. Fed after the E-stop, so the servos go limp on E-stop. |

Pins to leave unwired:

| Pin | Why |
|---|---|
| CN5-6 (D13, PA5) | Drives LD2, the firmware's heartbeat LED |
| CN9-1 / CN9-2 (D0 / D1, PA3 / PA2) | USART2 to the ST-LINK virtual COM port (solder bridges SB13/SB14) |
| CN6-8 (VIN), CN10-8 (U5V) | Power inputs/outputs of the board's own supply; feeding them fights the USB supply |

### Why the ECHO dividers

- HC-SR04 ECHO swings 0-5 V [[hcsr04]]. The STM32F401 runs at 3.3 V.
- The pins used here (PA0, PA1, PA4, PA8, PA9) are FT (5 V tolerant) in digital mode,
  but not in analog mode [[f401ds]]. The 1 kΩ / 2 kΩ divider (5 V x 2/3 = 3.33 V)
  keeps margin anyway, e.g. if firmware ever leaves a pin in analog mode.
- **The order matters.** Along the wire it is: Nucleo pin, divider node (2 kΩ to
  GND), 1 kΩ, sensor ECHO. If the 2 kΩ is joined on the sensor side of the 1 kΩ
  instead, it only loads the sensor output, and the pin gets the full 5 V pulse
  through the 1 kΩ.
- The input high threshold is 0.45 x VDD + 0.3 = 1.79 V at 3.3 V [[f401ds]]. 3.33 V
  is well above it.
- TRIG needs no level shifter. The HC-SR04 wants a 10 µs TTL pulse, and 3.3 V is a
  valid TTL high.

### Ultrasonic sensors

- HC-SR04: 5 V, about 15 mA, 40 kHz, 2 cm to 4 m, 15° cone. Allow 60 ms or more between
  pings [[hcsr04]]. The two sensors must be pinged one after the other, not at the
  same time, so neither hears the other's echo. That is a firmware requirement, not a
  wiring one.
- Mount LEFT at the front-left facing 45° outward, and RIGHT at the front-right facing
  45° outward. This matches `ultrasonic_left_link` and `ultrasonic_right_link` in the
  URDF (yaw +45° and -45°). They publish `/ultrasonic/left` (serial field `D1`) and
  `/ultrasonic/right` (`D2`).
- The ST-LINK USB budget is about 300 mA for the board and its 5V pin [[um1724]].
  Two sensors take about 30 mA.

### Servos (optional)

The gizmo actuator is **TBD**. This assumes two standard hobby servos.

- Standard servo plug colours: brown = -, red = +, orange = signal.
- PWM: 50 Hz, 1500 µs = 0°, ±1000 µs = ±90°. The firmware clamps pulses to 500-2500 µs.
- Enable with the launch argument `gizmo_mode:=servo`. The Pi then sends `G:<yaw>,<pitch>`
  lines to the Nucleo.
- Power the servos from the BEC only. A stalled servo draws amps, far more than the
  Nucleo 5V pin can supply. The BEC's OUT- must be joined to the Nucleo GND.

## 4. Camera ribbon

![Camera ribbon](wiring/05-camera-ribbon.svg)

All Raspberry Pi cameras use the standard 15-pin, 1.0 mm pitch connector. A Pi 5 has
two mini 22-pin, 0.5 mm pitch ports (CAM/DISP0 and CAM/DISP1). You can use either;
they are "by the edge closest to you between the micro HDMI connector and the
Ethernet port" [[rpicam]]. A Pi 5 therefore needs the **Standard-Mini** cable
(200 / 300 / 500 mm) [[rpicable]]. For a pan/tilt gizmo, take 300 or 500 mm.

| From | Pin | To | Pin | Wire | Notes |
|---|---|---|---|---|---|
| Camera Module v2.1 | 15-pin CSI connector | Standard-Mini cable | wide 15-pin end | ribbon | Contacts face away from the flap, which means towards the camera board |
| Standard-Mini cable | narrow 22-pin end | Raspberry Pi 5 | CAM/DISP0 or CAM/DISP1 | ribbon | Contacts face away from the flap |
| Camera Module v2.1 (Pi 4 or older) | 15-pin CSI connector | Raspberry Pi 4 | CAMERA (between the micro HDMI port and the audio jack) | Standard-Standard ribbon (supplied) | Same rule: contacts face away from the flap |

Procedure [[rpicam]]:

1. Shut the Pi down and disconnect its power. The camera ports are not hot-pluggable.
2. Earth yourself first. The camera is sensitive to static.
3. Open the connector flap: pull it out until it stops, then tilt it slightly away
   from the opening.
4. Insert the cable straight and fully, metallic contacts facing away from the flap.
   Push the flap back down until it clicks.
5. Do the same at the camera end. The camera connector is on the side opposite the
   lens.
6. If you fit the Pi 5 Active Cooler, plug the cable in first. The ports are hard to
   reach afterwards.
7. Boot and run `rpicam-hello --list-cameras`. It should list `imx219`.

On the gizmo, leave a loose service loop so the ribbon never pulls at full pan and
tilt. Let it bend along its length, never fold it sharply, and keep it away from the
motor and servo wires.

## Safety checklist before first power-on

Work through this with the battery **disconnected** unless a step says otherwise.

**Wiring checks (no power)**

- [ ] Every XT30 and the battery connector: + and - match the housing markings. Check
      with a multimeter, not by wire colour.
- [ ] F1 (15 A) is fitted within 10 cm of battery +. F2 (3 A) sits at J1 before the
      buck. With the servo BEC: F3 (3 A) sits at J2. With relay K1: F4 (1 A) sits at J1
      in the coil loop.
- [ ] The 0 V terminal block is the star point: battery negative, both motor returns,
      the buck IN-, the BEC IN- and the CAN adapter GND each have their own wire to it.
- [ ] No short on either branch. Measure + to 0 V at J1 and at J2: the reading should
      start low and climb as the drive capacitors charge, not stay near 0 Ω.
- [ ] SW1 off: J1 is isolated from battery +.
- [ ] E-stop released: S1 conducts. Pressed: S1 is open and stays open (latching).
- [ ] CAN_H to CAN_L reads about 60 Ω with everything plugged together (see
      [Termination check](#termination-check)).
- [ ] Continuity between battery -, the buck IN-, the buck OUT-, the Nucleo GND and the
      CAN adapter GND wire.
- [ ] No Nucleo pin other than CN6-5 and CN6-6 connects to a supply rail. CN6-8 (VIN)
      and CN10-8 (U5V) are unused.
- [ ] Motor CAN IDs are set to 1 (LEFT) and 2 (RIGHT) and labelled.
- [ ] Camera ribbon seated at both ends with both flaps closed.

**First power, Pi disconnected**

- [ ] Connect the battery with SW1 off, then switch SW1 on with the E-stop **pressed**.
- [ ] Buck output measures 5.1 V with no load. Adjust before connecting the Pi.
- [ ] Optional battery sense: the voltage at the Nucleo end of R1/R2 is about
      V(battery) / 11 (2.29 V at 25.2 V) before you plug it into CN8-3.
- [ ] Optional servo BEC: output measures the servo voltage (5-6 V, check the servo
      rating) before you plug the servos in.
- [ ] Switch SW1 off and connect the Pi.

**First power, everything connected**

- [ ] Wheels off the ground (robot on blocks), E-stop within reach.
- [ ] SW1 on with the E-stop pressed: the Pi boots and no undervoltage warning appears
      (`vcgencmd get_throttled` returns `throttled=0x0`).
- [ ] With the sensors powered, the divider nodes (Nucleo CN8-2 and CN5-1) never
      exceed 3.4 V.
- [ ] Release the E-stop: both drive LEDs light blue.
      `pixi run -e robot ros2 run iot_robot_drivers cubemars_tool check` finds IDs 1
      and 2, and the motors do not move on their own.
- [ ] With the robot software not running, start a slow 20 s jog:
      `pixi run -e robot ros2 run iot_robot_drivers cubemars_tool jog --id 1 --velocity 1.0 --duration 20`.
      Press the E-stop while the wheel turns: the wheel coasts to a stop and the Pi
      stays up. Keep S1 pressed until the jog command has exited, then release it. The
      wheel must stay still. Releasing S1 while the jog still runs would restart the
      wheel at once, which is the hazard described in the
      [E-stop warning](#1-power-distribution).

**Every session**

- [ ] Power on: SW1 on with the E-stop pressed, let the Pi boot, then release the
      E-stop. Nothing commands motion yet at that point. The robot service, if
      installed, keeps retrying its CAN check until the drives answer.
- [ ] After any E-stop press: stop the follower and teleop, release S1, let the drives
      boot, and only then restart the robot software. See the
      [E-stop warning](#1-power-distribution).
- [ ] Power off: press the E-stop, shut the Pi down (`sudo shutdown -h now`), then SW1
      off. Switching SW1 off under a running Pi risks corrupting the SD card.
- [ ] Charge the LiPo with a balance charger, in a LiPo bag, attended. Stop using it
      at 21 V (3.5 V per cell).

## Open items (TBD)

| Item | Assumed here | What changes if different |
|---|---|---|
| AK45-10 version | Original: separate XT30PW-M power and A1257WR-S-4P CAN ports | V3.0: one XT30(2+2) per motor, spliced bus, see [V3.0 variant](#ak45-10-v30-variant) |
| Built-in CAN terminator in the drive | None | If present (a bare drive reads about 120 Ω), both drives have one: drop R_T **and** switch the adapter's terminator off, then check for 60 Ω. See [Termination check](#termination-check). |
| USB-CAN adapter model | Native SocketCAN (gs_usb), switchable 120 Ω, not isolated | slcan: bring up with `deploy/scripts/can_up.sh`. No terminator: add a second 120 Ω at the adapter end. Isolated: the GND wire becomes mandatory. |
| Raspberry Pi model | Pi 5 | Pi 4: 3 A buck, Standard-Standard ribbon, CAMERA port, no `PSU_MAX_CURRENT` |
| Buck converter rating | 5.1 V, 5 A, 30 V+ input, non-isolated | Under 5 A: a Pi 5 may throttle under load. Input under 30 V: not safe on 6S. |
| Battery capacity and plug | 6S LiPo, plug unknown (often XT60) | Only the battery-side connector |
| Gizmo actuator | Two hobby servos on PA6/PA7 with a separate BEC | Anything else needs its own section |
| E-stop contact rating | 10 A DC or more | Lower: use the relay (option C) |
| Hardware E-stop feedback to ROS | None directly; the motor plugin latches a stop when the unpowered drives go silent | Possible later: a second contact block on S1 wired to a Nucleo input, reported by `stm32_bridge`, latching `/e_stop`. Needs firmware and driver support. |

## Regenerating the diagrams

| File | Source | How |
|---|---|---|
| `01-system-overview.svg`, `02-power-distribution.svg`, `03-can-bus.svg`, `05-camera-ribbon.svg` | Hand-written SVG | Edit the SVG directly |
| `04-nucleo.svg` | `docs/wiring/src/04-nucleo.py` (Python standard library only) | `python3 docs/wiring/src/04-nucleo.py` |
| `03-can-bus-harness.svg` | `docs/wiring/src/03-can-bus-harness.yml` (WireViz 0.4) | `uvx wireviz -f s -o docs/wiring docs/wiring/src/03-can-bus-harness.yml` |

- WireViz needs Graphviz (`dot`) on the PATH.
- `uvx` runs WireViz in a throwaway environment, so nothing is installed into the
  project.
- If you change a pin assignment, change it in the diagram, in this file, and in the
  STM32 firmware or launch arguments together.

To check a diagram for overlapping or clipped labels, render it to PNG with headless
Chrome and look at the image. Set the window size to the SVG's `width` and `height`:

```bash
google-chrome --headless=new --disable-gpu --hide-scrollbars \
  --screenshot=/tmp/04-nucleo.png --window-size=1600,1110 \
  "file://$PWD/docs/wiring/04-nucleo.svg"
```

## Sources

Citations in the text, such as [[akdrv]], link straight to the document listed here.

| Key | Source | Used for |
|---|---|---|
| `ak45` | CubeMars, [AK45-10 KV75 product page][ak45] | Voltage, rated and peak current, torque |
| `ak45v3` | CubeMars, [AK45-10 V3.0 KV75 product page][ak45v3] | V3.0 single-connector variant |
| `akdrv` | CubeMars, [AK Series Driver Manual V1.0.18][akdrv] | Connector models and pinouts (section 1.2), indicator LEDs (1.3), accessories and the 30 AWG CAN lead (1.4), R-Link (2), 18-28 V / 16-28 V input |
| `akv3` | CubeMars, [AK Series Module Product Manual v3.0.0][akv3] | XT30(2+2) pinout and supplied pigtail |
| `ak40` | CubeMars, [AK40-2410 drive installation instructions][ak40] | Power lead colours (inconsistent, see the power section) |
| `a1257` | CJT, [A1257 series 1.25 mm wire-to-board connector datasheet][a1257] | Crimp terminal wire range (AWG 28-32, insulation 1.0 mm OD max), 1 A rating |
| `cmhw` | OpenFieldAutomation, [`cubemars_hardware` README][cmhw] and its `src/system.cpp` (vendored in `src/external/cubemars_hardware`) | CAN bitrate, status frames; keeps sending the last command when a motor does not answer |
| `um1724` | STMicroelectronics, [UM1724 STM32 Nucleo-64 boards (MB1136) user manual, Rev 17][um1724] | Header pinout (Figure 17, Tables 16 and 29), JP5, SB13/SB14, USB power budget |
| `f401ds` | STMicroelectronics, [STM32F401xD/xE datasheet, DocID025644 Rev 3][f401ds] | FT (5 V tolerant) pins, input thresholds |
| `hcsr04` | Elecfreaks, [HC-SR04 Ultrasonic Ranging Module datasheet][hcsr04] | Supply, range, trigger pulse, ping interval |
| `rpipower` | Raspberry Pi documentation, [Power supply][rpipower] | PSU ratings, USB current limits, 4.63 V warning, camera current |
| `rpieeprom` | Raspberry Pi documentation, [bootloader EEPROM: `PSU_MAX_CURRENT`][rpieeprom] | Pi 5 on a non-PD supply |
| `rpiconfig` | Raspberry Pi documentation, [configuration reference][rpiconfig] | `usb_max_current_enable` |
| `rpicam` | Raspberry Pi documentation, [Install a Raspberry Pi camera][rpicam] | Connector locations, ribbon orientation, procedure |
| `rpicable` | Raspberry Pi, [Camera cable product page][rpicable] | Standard-Mini cable lengths |
| `bret` | bret.dk, [How to power the Raspberry Pi 5: a complete guide][bret] | GPIO powering bypasses the input protection |
| `kvaser` | Kvaser, [How to test your CAN termination works correctly][kvaser] | 60 / 120 / 40 Ω readings |
| `inno` | Inno-maker, [USB2CAN documentation][inno] and [product page](https://www.inno-maker.com/product/usb2can-core/) | gs_usb driver, DB9 pinout, 120 Ω jumper |
| - | [WireViz](https://github.com/wireviz/WireViz) | Harness drawing tool |

[ak45]: https://www.cubemars.com/product/AK45-10-robotic-actuatuor.html
[ak45v3]: https://www.cubemars.com/product/ak45-10-v3-0-kv75-robotic-actuator.html
[akdrv]: https://www.cubemars.com/data/cms/202605/ak-series-driver-manual-v1-0-18-for-ak-2-0-robotic-actuator.pdf
[akv3]: https://img.cubemars.com/products/cubemars-product-parameter/AK-Series-Module-Product-Manual-v3.0.0-Download.pdf
[ak40]: https://www.cubemars.com/data/cms/202607/ak40-2410-1a-a1-drive-installation-instructions.pdf
[cmhw]: https://github.com/OpenFieldAutomation-OFA/cubemars_hardware
[um1724]: https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf
[f401ds]: https://www.st.com/resource/en/datasheet/stm32f401re.pdf
[hcsr04]: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
[rpipower]: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
[rpieeprom]: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#PSU_MAX_CURRENT
[rpiconfig]: https://www.raspberrypi.com/documentation/computers/configuration.html
[rpicam]: https://www.raspberrypi.com/documentation/accessories/camera.html#install-a-raspberry-pi-camera
[rpicable]: https://www.raspberrypi.com/products/camera-cable/
[bret]: https://bret.dk/how-to-power-the-raspberry-pi-5-a-complete-guide/
[kvaser]: https://kvaser.com/developer-blog/how-to-test-your-can-termination-works-correctly/
[a1257]: https://cjt.com/upload/A1257.pdf
[inno]: https://github.com/INNO-MAKER/usb2can
