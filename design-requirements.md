# Nullvember 2.0 — mBot Design Requirements

> Status: **BUILT, see `NullvemberBot/NullvemberBot.ino`.** Values marked "calibrate" are starting guesses to tune on the real course using the calibration modes (§10).
> Last updated: 2026-11-14

## 1. Hardware

| Item | Detail | Port | Notes |
|---|---|---|---|
| Controller | mBot 1 (mCore, ATmega328) | — | Onboard button, 2× RGB LED, buzzer available for feedback |
| Drive | 2× DC motors, differential drive | M1 (left), M2 (right) | Left motor is mirrored, so its speed sign is inverted in code. **No encoders and no gyro**, so all turns are **timed** |
| Ultrasonic | Me Ultrasonic Sensor, **facing RIGHT** | TBD (default Port 3) | Measures distance to the right wall only. The course never puts a wall ahead (§3) |
| Line follower | Me Line Follower (2 IR sensors, S1/S2), under the front | TBD (default Port 2) | Returns 0 = both black, 1 = right black, 2 = left black, 3 = both white |
| Power | Organiser-supplied batteries only | — | Timed turns drift as the battery drains, so re-calibrate on fresh batteries |

Rule constraint: nothing that isn't organiser-supplied may be on the robot during a scored run.

## 2. Software platform

- **Arduino IDE** with the Makeblock `MeMCore` library (board: Arduino Uno).
- All tunable values go in **named constants at the top of the file** so they can be changed quickly between runs.

## 3. Course layout (from photos in `course-photos/`)

- The **overview photo is the reference layout.** The short stub beside the divider in the zoomed photo is not meant to be there.
- Two parallel corridors, each **~40 cm wide**, separated by a divider wall. The divider ends before the far end wall, which leaves the U-turn gap.
- **Corridor A (start):** the robot starts at the black arrow and drives along it with the outer wall on its **right**. The corridor is roughly straight, and the robot never meets a wall ahead.
- **Line #1:** black tape across corridor A. Triggers the spin challenge.
- **Line #2:** black tape across corridor A near the far end. It runs under the end of the divider into corridor B and forms the **crossbar of the T**.
- **U-turn:** around the end of the divider into corridor B (left, left).
- **Corridor B:** a single line-following path branches off the crossbar and runs back up corridor B with sharp-ish corners. It ends at a small crossbar near the top.
- Tape is **~2 cm wide**, on a light wood floor.

## 4. Behaviour overview (state machine)

```
[SELECT_START] --1 press--> [WALL_FOLLOW_1]   (from the start)
               --2 presses-> [WALL_FOLLOW_2]   (checkpoint: after line #1)
               --3 presses-> [U_TURN]          (checkpoint: at line #2)

[WALL_FOLLOW_1]  --line #1--> [SPIN_3x360_CW] --> [CLEAR_LINE] --> [WALL_FOLLOW_2]
[WALL_FOLLOW_2]  --line #2--> [U_TURN: F1, L1, F2, L2, F3] --> [SWEEP_SEARCH]
[SWEEP_SEARCH]   --line found--> [LINE_FOLLOW]
[LINE_FOLLOW]    --line lost for LINE_END_MS--> [STOP]
```

- There are **only two counted lines.** The counter goes up only in a WALL_FOLLOW state, on a white→black edge (debounced).
- **Line detection is ignored for the first `START_IGNORE_MS` (2 s) after starting**, so the start arrow is never counted.

## 5. Behaviour requirements

### 5.1 Wall following (WALL_FOLLOW_1/2), right-hand wall, nudge steering
- Target **15 cm** from the right wall. Steering uses **fixed nudges**: one wheel slows by `NUDGE` while the other stays at `BASE_SPEED`.

| Distance `d` | Action |
|---|---|
| `d` < 15 − tol | too close, nudge **left** |
| 15 − tol ≤ `d` ≤ 15 + tol | **straight** |
| 15 + tol < `d` ≤ 25 | too far, nudge **right** |
| `d` > 25, or `d` = 0 (no echo) | **null reading, drive straight** |

| Constant | Meaning | Value |
|---|---|---|
| `WALL_TARGET_CM` | desired distance to right wall | 15 |
| `WALL_TOLERANCE_CM` | dead-band | 2 (adjustable) |
| `WALL_NULL_CM` | beyond this = null reading, drive straight | 25 |
| `BASE_SPEED` | cruising PWM (0–255) | calibrate |
| `NUDGE` | speed taken off one wheel to steer | calibrate |

### 5.2 Line #1: three clockwise 360° spins (timed only)
- Stop, then **spin in place clockwise** (left wheel forward, right wheel backward) as **one continuous timed spin** of 1080°.
- Exact final heading depends on calibrating `SPIN_TOTAL_MS` on the real floor. Wall following corrects small leftover errors.
- Then clear the line (drive until both sensors read white) and resume wall following.

| Constant | Meaning | Value |
|---|---|---|
| `SPIN_SPEED` | PWM during spin | calibrate |
| `SPIN_360_MS` | time for one 360° at `SPIN_SPEED` | calibrate |
| `SPIN_COUNT` | number of spins | 3 |
| `SPIN_TOTAL_MS` | `SPIN_360_MS × SPIN_COUNT` (overridable if 3 × 1 isn't exact) | derived |

### 5.3 Line #2: U-turn as Forward → Left → Forward → Left → Forward, with nudging
- The forward segments **nudge off the right-hand wall** (same nudge logic as §5.1) so they stay straight. Null readings (> 25 cm) mean drive straight.
- The two 90° lefts are **timed pivots in place** (no nudging possible while turning).
- F1 and F2 ignore the line sensor. **F3 ends when it detects the crossbar** (line #2 again, now in corridor B), with a 30 cm timed limit as a fallback.
- Distances are measured from where the robot **stops** on detecting line #2. The sensors are at the front, so the wheels are ~7 cm behind the tape.

| Constant | Segment | What's on the right / nudge target | Value |
|---|---|---|---|
| `UT_FWD1_MS` | Forward 1, 30 cm | corridor A's outer wall, 15 cm | calibrate to 30 cm |
| `UT_LEFT1_MS` | Left 90° pivot | — | calibrate |
| `UT_FWD2_MS` | Forward 2, 35 cm | the end wall; target **TBD (§9)** | calibrate to 35 cm |
| `UT_LEFT2_MS` | Left 90° pivot | — | calibrate |
| `UT_FWD3_MAX_MS` | Forward 3, until crossbar (max ~30 cm) | corridor B's outer wall; target **TBD (§9)** | calibrate to 30 cm |
| `UT_SPEED` / `UT_TURN_SPEED` | PWM | | calibrate |

### 5.4 Sweep search
- Entry: F3 has just detected the crossbar. The robot creeps forward to cross it:
  - sensors go white within `CROSSBAR_CLEAR_MS` → crossbar cleared, start the zig-zag;
  - sensors still black after `CROSSBAR_CLEAR_MS` → it's already on the stem of the T, so **skip the sweep and go straight to line following**.
- The branch's side is unknown, so the robot creeps forward while zig-zagging **left/right, getting wider each swing** up to a maximum.
- Only accept a line seen **after** the sensors have read white for `SWEEP_ARM_MS`, so the crossbar can't trigger it.
- Line found = either sensor sees black. Then hand over to line following.
- **Failsafe:** nothing found within `SWEEP_TIMEOUT_MS` → **stop, red LED**, and the reset-er uses a reset.

| Constant | Meaning | Value |
|---|---|---|
| `SWEEP_STEP_MS` | width of the first swing | calibrate |
| `SWEEP_GROW_MS` | how much wider each swing gets | calibrate |
| `SWEEP_MAX_MS` | widest swing allowed | calibrate |
| `SWEEP_CREEP_MS` | forward creep between swings | calibrate |
| `SWEEP_ARM_MS` | white time needed before a line counts | calibrate |
| `SWEEP_FIRST_DIR` | side to try first | right (adjustable) |
| `CROSSBAR_CLEAR_MS` | longest time to cross 2 cm tape | calibrate |
| `SWEEP_TIMEOUT_MS` | give up and stop | TBD (suggest 8000) |

### 5.5 Line following
- Aim: **keep both sensors on black** (reading 0).

| Reading | Meaning | Action |
|---|---|---|
| 0 (both black) | centred | forward |
| 1 (right on black) | drifting left | steer right |
| 2 (left on black) | drifting right | steer left |
| 3 (both white) | lost | pivot toward the **last side seen**, for at most `LF_SEARCH_MAX_MS` (~100°), then wait |

- The pivot limit stops the robot turning right round at the end of the track and following the line back the way it came.
- **Finish:** line lost continuously for **`LINE_END_MS` = 2000 ms** → stop (flashing green + beeps).

| Constant | Meaning | Value |
|---|---|---|
| `LF_SPEED` | straight PWM | calibrate |
| `LF_TURN_SPEED` | correction PWM | calibrate |
| `LINE_END_MS` | lost this long = finished | **2000** (adjustable) |

## 6. Line-detection rules (counting lines #1 and #2)
- Black tape, ~2 cm wide, light floor.
- **Detection = either sensor on black.**
- Debounce: stable for `LINE_DEBOUNCE_MS` (~20 ms).
- **Ignored for `START_IGNORE_MS` = 2000 ms after starting** (start arrow).
- After an action, wait until both sensors read white before counting can resume.

## 7. Start / checkpoint selection
- Power on, then press the onboard button **1, 2 or 3 times**:
  - **1 press:** start from the beginning;
  - **2 presses:** start after line #1;
  - **3 presses:** start the U-turn. The reset-er places the robot **on line #2, facing down corridor A**, where it would have stopped on detecting the line.
- Each press restarts a **2 s** timer (`START_DELAY_MS`). When 2 s pass with no press, the count is final and the robot starts moving. This also gives the reset-er time to take their hand away.

## 8. Competition-run notes
- 5-minute limit to complete one full run, so the program must be reliable first and fast second.
- Declare challenge choices before each run. Declare LLM-generated code before scoring.

## 9. Resolved decisions (latest round)
- U-turn F2 and F3 nudge targets: **hold the distance read at the start of the stretch** (null if > 25 cm).
- Stem of the T: **~10 cm from corridor B's outer wall**, so the zig-zag should reach ~10 cm either side (`SWEEP_MAX_MS`).
- LEDs + beep per stage: **yes**. Colours: dim cyan idle, green wall, blue spin, purple U-turn, yellow sweep, white line follow, flashing green finished, red stopped.
- Emergency stop: pressing the button during a run stops the robot (touching it is a reset under the rules anyway).

## 10. Calibration procedure (do in this order, on the real floor, fresh batteries)

| Presses | Mode | What to do | Adjust |
|---|---|---|---|
| 7 | Sensor monitor (Serial 115200) | Tape under the left sensor only, which should show `L=1 R=0`. Hold a wall at 15 cm and check the distance reading | `SWAP_LINE_SENSORS`, ports |
| 6 | Distance test | Robot drives `CAL_DIST_CM` (100 cm). Measure actual D | `MS_PER_CM = MS_PER_CM × 100 / D`. If it spins/reverses, fix `INVERT_*_MOTOR` |
| 5 | 90° test | Four left pivots, which should end facing the start | `TURN90_MS` |
| 4 | Spin test | 3 × 360° clockwise, which should end facing the start exactly | `SPIN_360_MS` (or `SPIN_TOTAL_OVERRIDE_MS`) |
| 8 | Wall-follow only | Watch it track 15 cm. Wobbly → lower `NUDGE`; drifts → raise it | `NUDGE`, `BASE_SPEED`, `WALL_TOLERANCE_CM` |
| 9 | Line-follow only | Run the corridor B track. Check that sharp corners recover and it stops at the end | `LF_*`, `LF_SEARCH_MAX_MS`, `LINE_END_MS` |
| 3 | U-turn checkpoint | Robot on line #2 facing down corridor A. Check that it lands in corridor B and finds the stem | `UT_FWD*_CM`, `SWEEP_*`, `CROSSBAR_CLEAR_MS` |
| 1 | Full run | End-to-end. Time it: must finish well within 5 min | — |

Re-check modes 4, 5 and 6 whenever the batteries are swapped, because timed moves change with battery voltage.
