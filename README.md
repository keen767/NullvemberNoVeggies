# Nullvember 2.0 — mBot maze robot (team NoVeggies)

mBot 1 program for the IEEE-NUS Nullvember 2.0 maze hackathon.

- **Sketch:** [`NullvemberBot/NullvemberBot.ino`](NullvemberBot/NullvemberBot.ino), for Arduino IDE with the Makeblock `MeMCore` library (board: Arduino Uno)
- **Design + calibration guide:** [`design-requirements.md`](design-requirements.md)
- **Course photos:** [`course-photos/`](course-photos/)

## What it does
1. Follows the right wall at 15 cm (ultrasonic facing right, nudge steering).
2. At line #1, spins clockwise 3 × 360°, then keeps wall-following.
3. At line #2, does a U-turn (forward 30 → left 90 → forward 35 → left 90 → forward until the crossbar).
4. Zig-zags to find the stem of the T, then follows the line until it has been lost for 2 s.

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
