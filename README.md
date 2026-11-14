# Nullvember 2.0 — mBot maze robot (team NoVeggies)

mBot 1 program for the IEEE-NUS Nullvember 2.0 maze hackathon.

- **Sketch:** [`NullvemberBot/NullvemberBot.ino`](NullvemberBot/NullvemberBot.ino), for Arduino IDE with the Makeblock `MeMCore` library (board: Arduino Uno)
- **Design + calibration guide:** [`design-requirements.md`](design-requirements.md)
- **Course photos:** [`course-photos/`](course-photos/)

## What it does
1. Ignores lines for the first 2 s (start arrow), then follows the right wall at 12 cm (ultrasonic on port 1 facing right, nudge steering). Readings over 100 cm count as "no wall" and it drives straight.
2. At line #1, spins clockwise 3 × 360° in place (timed only, `SPIN_360_MS` = 1840 ms per spin), then keeps wall-following.
3. At line #2, does a U-turn with nudging: forward 34 cm → left 90° → forward 26 cm → left 90° → forward (max 30 cm) until the crossbar.
4. Zig-zags to find the stem of the T. It swings left first, and left swings are 130% as long as right ones. It stops (red) if nothing is found in 8 s.
5. Follows the line, trying to keep both sensors on black, and finishes once the line has been lost for 2 s.

## LED feedback while wall-following
| LED | Meaning |
|---|---|
| Blue | Nudging left (too close to the wall) |
| Yellow | Nudging right (too far from the wall) |
| Green | Going straight (within ±1 cm of target) |
| Off | No wall seen (driving straight) |

All tuning values are constants near the top of the sketch. The change log is in the header comment.

## Button modes
Press the onboard button N times; the robot starts 2 s after the last press.

| Presses | Mode |
|---|---|
| 1 | Full run |
| 2 | Start after line #1 |
| 3 | Start the U-turn (robot on line #2) |
| 4 | Spin calibration |
| 5 | 90° turn calibration |
| 6 | Distance calibration |
| 7 | Sensor monitor (Serial 115200) |
| 8 | Wall-follow only |
| 9 | Line-follow only |

## LLM declaration
Code was generated with Claude (Anthropic) from the team's design requirements, then tuned and tested by the team.
