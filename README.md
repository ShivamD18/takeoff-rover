# TakeOFF Rover firmware (rebuilt from the CAD, 2026-11-26)

ESP32 + Xbox Wireless controller (Bluepad32). The arm geometry comes from the Fusion
assembly exported as `TakeOFF_Rover.stp`, plus one measurement taken on the built rover.

| Folder | What it is |
|---|---|
| `TakeOffRover/` | Rover firmware: drive, arm IK with deck/bracket keep-out, presets, claw, failsafe |
| `ServoBenchTest/` | Servo bench test and calibration over Serial (`s`/`e`/`c <µs>`, `x`, `t`, `h`) |
| `tools/arm_check.cpp` | PC test of the arm math: presets, moves between them, pulse ranges |
| `pages/takeoff-arm-sim.html` | Offline arm simulator, any browser. Same geometry, limits, keep-out and presets as the firmware |
| `pages/takeoff-arm-live.html` | Same simulator plus a USB link: with `ServoBenchTest` flashed, the virtual arm drives the real servos (Chrome, Edge or Brave) |
| `cad/TakeOFF_Rover.stp.zip` | Fusion assembly as STEP (zipped; 55 MB unzipped). The arm numbers in the firmware come from this |
| `archive/` | The original handoff zips the project started from (old firmware, kept for reference only) |

Both sketches compile with **esp32_bluepad32 4.1.0**, board **ESP32 Dev Module**. `ServoBenchTest` is running on the
rover; `TakeOffRover` has not been flashed yet (it waits on the arm calibration below).

## Web pages

Open either page straight from the folder (double-click it, no server needed).

**Simulator** (`pages/takeoff-arm-sim.html`), shown at the Pick preset. Drag the claw tip or press a preset; the
joint-space chart shows every pose the firmware allows (red cells would hit the deck, bracket or floor).

![Arm simulator at the Pick preset](docs/images/sim-pick.png)

**Live page** (`pages/takeoff-arm-live.html`), shown with raw calibration mode ticked before connecting, so every servo
is still limp. Connect to ESP32 talks to `ServoBenchTest` over USB.

![Live page in raw calibration mode](docs/images/live-raw.png)

## Wiring

| Part | GPIO |
|---|---|
| Shoulder MG996R (horn carries the 120 mm upper arm) | **19** |
| Elbow MG996R (horn carries the 30 mm crank → push rod → forearm) | **18** |
| Claw SG90 | **21** |
| L298N ENA / IN1 / IN2 | 25 / 26 / 27 (not re-checked since the rewire) |
| L298N ENB / IN3 / IN4 | 33 / 32 / 13 (not re-checked since the rewire) |
| Battery sense (only with the divider fitted; set `HAVE_VBAT_SENSE`) | 34 |

## Arm geometry

Origin on the shoulder servo axis, x forward, z up. The claw tip is the claw centre at the bottom of the fingers.

| Value | mm | Source |
|---|---|---|
| Upper arm / forearm / crank / push rod | 120 / 100 / 30 / 120 | STEP |
| Shoulder axis above the floor | 152.4 | measured (~6 in); CAD says 160.2 |
| Wrist pin → finger bottom | 84.3 | STEP |
| Claw centre ahead of the wrist pin | 15.7 | STEP |
| Deck top below the shoulder axis | 39.5 | STEP |
| Deck front edge ahead of the shoulder axis | 75.0 | STEP |
| Servo bracket: forward reach / top above the shoulder axis | 28.6 / 25.8 | STEP |
| Jaw gap fully open | 45.8 | STEP |

Joint limits: upper arm −10…100°, forearm −85…25°, fold angle 20…160°. Inside those limits the
fingers reach the floor anywhere from about **140 to 222 mm** ahead of the shoulder axis.

Presets (x ahead of the shoulder axis, z = finger bottom above the floor):
Pick (170, 5) · Carry (140, 60) · Drop over wall (200, 55) · Stow (115, 120).
The Stow preset matches the pose the CAD is drawn in. `tools/arm_check` confirms each preset is ≥17° inside every joint limit
and that all 12 moves between presets stay clear of the deck and bracket.

## Bring-up

1. **Horns off.** Flash `ServoBenchTest`, Serial Monitor 115200 with Newline, type `h`.
2. `c 1500`, `s 1500`, `e 1500`: each servo moves and holds. Watch the servo rail voltage (sag below ~4.6 V = regulator too weak).
3. Fit the horns at 1500 µs so the upper arm sits **45° above** forward-horizontal and the forearm **30° below** it.
4. **Direction:** `s 1550` should tip the upper arm up/back; `e 1450` should lift the front of the forearm.
   The two servos face each other, so the signs in `arm_kinematics.h` are opposite (`SHOULDER +10`, `ELBOW −10`).
   If one is backwards, flip **both**.
5. Claw: find the open/closed pulses and measure the gap at 1500 µs → `claw::REF_GAP`, `OPEN_SIGN`, `MIN_US`/`MAX_US`.
6. Flash `TakeOffRover` with the wheels off the ground. Check pairing, that switching the controller off stops the motors,
   and each motor's direction (`LEFT_INVERT` / `RIGHT_INVERT`). Serial prints the tip position and pulses at 5 Hz.
7. Press **A** (Pick) on the floor. If the fingers don't sit ~5 mm above the floor, adjust `SHOULDER_H` in `arm_kinematics.h`
   and rerun `tools/arm_check`.

## Calibrating the arm with the live page (horns already fitted)

The horns are on at unknown angles, so measure each joint instead of refitting them. Angles are measured
from forward-horizontal (a phone level app or a set square against the deck works). Before you start:
ServoBenchTest is on the ESP32, the page is reloaded, and Brave/Chrome has the port (no Serial Monitor open).

1. Tick **Raw pulses for calibration**, *then* press Connect. The arm stays limp until you move a slider.
2. Hold the arm. **Elbow slider** first: forearm **level** (0°) → note `e`.
3. **Shoulder slider**: upper arm **straight up** (90°) → note `s`. Then upper arm **45° up, leaning forward** → note `s`.
4. **Elbow slider** again (upper arm still at 45°): forearm pointing **45° down** at the front → note `e`.
5. **Claw slider**: jaws fully open (stop before it strains) → note `c`; jaws closed on a LEGO brick → note `c`.
6. Move shoulder and elbow to the **stow pose** you want → note `s` and `e`.

Always keep the short crank well away from lining up with the upper arm: that is where the linkage goes flat and twists.
The `s … · e … · c …` line under the checkbox selects with one click, so you can copy it. Disconnect no longer resets the board, so the servos
keep holding after you disconnect.

From these numbers: shoulder µs/° = (s₉₀ − s₄₅)/45 and `SHOULDER.us0` = s₄₅; elbow µs/° = (e₋₄₅ − e₀)/−45 and
`ELBOW.us0` = e₀ − 30 × that rate. Stow comes from the stow pulses through the forward kinematics.

Run the PC check after changing any geometry or preset:

```bash
g++ -std=c++17 -O2 -Wall -ITakeOffRover tools/arm_check.cpp -o /tmp/arm_check && /tmp/arm_check
```

## Not done yet

- If you change geometry, presets or servo signs in `arm_kinematics.h` / `config.h`, make the same change at the
  top of the script in both pages (they keep their own copy of the numbers).
- Battery type, regulator rating, motor pins and minimum motor PWM are unconfirmed.
- The keep-out models the deck, the bracket and the floor only. It doesn't cover the wheels (they sit outside the arm's
  line) or the claw folding back into the arm's own links.
