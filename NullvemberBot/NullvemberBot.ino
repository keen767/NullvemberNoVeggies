/*
 * Nullvember 2.0 — mBot 1 maze program
 * ---------------------------------------------------------------
 * Design spec: ../design-requirements.md
 *
 * LLM DECLARATION (required by the competition rules):
 *   This sketch was generated with Claude (Anthropic) from the team's own
 *   design requirements, then tuned/tested by the team.
 *
 * HOW TO USE
 *   Power on -> LEDs dim cyan (idle). Press the onboard button N times.
 *   2 s after the LAST press, the robot starts.
 *
 *   COMPETITION MODES
 *     1 press  : full run from the start arrow
 *     2 presses: checkpoint — start after line #1 (skip the spins)
 *     3 presses: checkpoint — start the U-turn (place robot ON line #2,
 *                facing down corridor A, where it would have stopped)
 *
 *   CALIBRATION / TEST MODES (press the button again to stop any of them)
 *     4 presses: spin test        — does the 3x360 clockwise spin
 *     5 presses: 90-degree test   — four left 90° pivots (should end facing start)
 *     6 presses: distance test    — drives CAL_DIST_CM straight at UT_SPEED
 *     7 presses: sensor monitor   — no motors; prints sensors over Serial (115200)
 *     8 presses: wall-follow only — tune WALL_* and NUDGE
 *     9 presses: line-follow only — place on the line; tune LF_*
 *
 *   Pressing the button during any run = emergency stop (red LEDs).
 *
 * LED COLOURS
 *   dim cyan = idle   green = wall following   blue = spinning
 *   purple = U-turn   yellow = sweep search    white = line following
 *   flashing green = finished   red = stopped/failed   orange = calibration
 */

#include <MeMCore.h>

// =====================================================================
//  PORTS
// =====================================================================
MeDCMotor          motorL(M1);
MeDCMotor          motorR(M2);
MeUltrasonicSensor ultra(PORT_3);       // mounted facing RIGHT
MeLineFollower     lineSensor(PORT_2);
MeRGBLed           led(0, 30);
MeBuzzer           buzzer;

// =====================================================================
//  TUNABLE SETTINGS — change these between runs
//  (values marked "calibrate" are starting guesses: tune on the course)
// =====================================================================

// ---- Motor / sensor wiring --------------------------------------------
const bool INVERT_LEFT_MOTOR  = true;   // standard mBot: left motor is mirrored
const bool INVERT_RIGHT_MOTOR = false;
const bool SWAP_LINE_SENSORS  = false;  // flip if mode 7 shows left/right reversed

// ---- General -------------------------------------------------------------
const unsigned long START_DELAY_MS    = 2000; // wait after last button press
const unsigned long START_IGNORE_MS   = 2000; // ignore lines after starting (start arrow)
const unsigned long STOP_SETTLE_MS    = 150;  // pause after braking so the robot stops rolling
const bool          BEEP_ON_STATE     = true; // short beep at every stage change
const uint8_t       LED_BRIGHTNESS    = 40;   // 0-255

// ---- Wall following (right wall, nudge steering) -------------------------
const float WALL_TARGET_CM    = 12.0;  // desired distance to the right wall
const float WALL_TOLERANCE_CM = 2.0;   // dead-band either side of the target
const float WALL_NULL_CM      = 25.0;  // further than this = null reading, drive straight
const int   BASE_SPEED        = 150;   // cruising PWM (0-255)          calibrate
const int   NUDGE             = 50;    // PWM taken off one wheel to steer  calibrate

// ---- Line detection (counting lines #1 and #2) ----------------------------
const unsigned long LINE_DEBOUNCE_MS = 15;  // black must persist this long to count
const unsigned long LINE_CLEAR_MS    = 100; // white needed after an action before counting again

// ---- Line #1: three clockwise 360° spins (timed only) ----------------------
const int           SPIN_SPEED  = 150;  // PWM during spin                  calibrate
const unsigned long SPIN_360_MS = 1800; // time for ONE 360° at SPIN_SPEED  calibrate (mode 4)
const int           SPIN_COUNT  = 3;
const unsigned long SPIN_TOTAL_OVERRIDE_MS = 0; // >0 = use this total instead of 360 x 3

// ---- Turns ------------------------------------------------------------------
const int           TURN_SPEED = 120;   // PWM for 90° pivots               calibrate
const unsigned long TURN90_MS  = 550;   // time for a 90° left pivot        calibrate (mode 5)

// ---- U-turn: F1 -> L90 -> F2 -> L90 -> F3 -------------------------------------
const int   UT_SPEED       = 150;   // PWM for the forward stretches
const float MS_PER_CM      = 200.0;  // ms to travel 1 cm at UT_SPEED      calibrate (mode 6)
const float UT_FWD1_CM     = 30.0;  // past the end of the divider (nudge off corridor A wall @15 cm)
const float UT_FWD2_CM     = 35.0;  // across the gap (nudge: hold starting distance)
const float UT_FWD3_MAX_CM = 30.0;  // up corridor B until the crossbar is seen (max distance)

// ---- Crossbar / sweep search -------------------------------------------------
const int           CREEP_SPEED       = 90;   // slow forward to cross the crossbar
const unsigned long CROSSBAR_CLEAR_MS = 400;  // still black after this = already on the stem
const int           SWEEP_SPEED       = 110;  // outer wheel PWM while sweeping
const int           SWEEP_INNER_SPEED = 0;    // inner wheel PWM while sweeping (0 = pivot on wheel)
const unsigned long SWEEP_STEP_MS     = 250;  // first half-swing                    calibrate
const unsigned long SWEEP_GROW_MS     = 150;  // each half-swing grows by this       calibrate
const unsigned long SWEEP_MAX_MS      = 700;  // widest half-swing (~10 cm reach)    calibrate
const unsigned long SWEEP_CREEP_MS    = 200;  // forward creep between swings        calibrate
const unsigned long SWEEP_ARM_MS      = 80;   // white needed before a line counts
const unsigned long SWEEP_TIMEOUT_MS  = 8000; // give up -> stop (red) -> use a reset
const int           SIDE_LEFT  = -1;
const int           SIDE_RIGHT = +1;
const int           SWEEP_FIRST_DIR   = SIDE_RIGHT;

// ---- Line following ------------------------------------------------------------
const int           LF_SPEED         = 120; // both sensors on black         calibrate
const int           LF_INNER_SPEED   = 30;  // inner wheel when correcting   calibrate
const int           LF_SEARCH_SPEED  = 110; // pivot speed when line is lost
const unsigned long LF_SEARCH_MAX_MS = 700; // max pivot when lost (~100°), so it can't turn round
const unsigned long LINE_END_MS      = 2000;// lost this long = course finished

// ---- Calibration ---------------------------------------------------------------
const float CAL_DIST_CM = 100.0;   // mode 6 drives this far: measure and adjust MS_PER_CM

// =====================================================================
//  LOW-LEVEL HELPERS
// =====================================================================
bool aborted = false;

bool buttonDown() { return analogRead(A7) < 100; }   // onboard button on A7

void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  motorL.run(INVERT_LEFT_MOTOR  ? -left  : left);
  motorR.run(INVERT_RIGHT_MOTOR ? -right : right);
}

void brake() {
  setMotors(0, 0);
  delay(STOP_SETTLE_MS);
}

// Emergency stop: pressing the button during a run.
bool checkAbort() {
  if (!aborted && buttonDown()) {
    aborted = true;
    setMotors(0, 0);
    Serial.println(F("** ABORTED by button **"));
  }
  return aborted;
}

void beep(uint16_t freq, uint32_t ms) {
  buzzer.tone(freq, ms);
  buzzer.noTone();
}

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  led.setColor(0, r, g, b);
  led.show();
}

void showState(const __FlashStringHelper *name, uint8_t r, uint8_t g, uint8_t b) {
  setLed(r, g, b);
  Serial.print(F("STATE: "));
  Serial.println(name);
  if (BEEP_ON_STATE) beep(1200, 60);
}

// Drive with fixed wheel speeds for a time. Returns false if aborted.
bool driveFor(int left, int right, unsigned long ms) {
  setMotors(left, right);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (checkAbort()) return false;
  }
  brake();
  return true;
}

bool pivotLeft90() {
  return driveFor(-TURN_SPEED, TURN_SPEED, TURN90_MS);
}

// ---- Ultrasonic -----------------------------------------------------------
// Returns distance in cm. With the installed Makeblock library, "no echo"
// comes back as 400 (> WALL_NULL_CM, so treated as null). The library only
// fires a new ping every ~23 ms and returns the cached value in between.
float readDistance() {
  return ultra.distanceCm();
}

bool isValidWall(float d) { return d > 0 && d <= WALL_NULL_CM; }

// ---- Line sensor ------------------------------------------------------------
// readSensors(): bit1 = S1, bit0 = S2, and a 1 bit means WHITE.
void readLine(bool &leftBlack, bool &rightBlack) {
  uint8_t v = lineSensor.readSensors();
  bool s1Black = !(v & 0x02);
  bool s2Black = !(v & 0x01);
  if (SWAP_LINE_SENSORS) { leftBlack = s2Black; rightBlack = s1Black; }
  else                   { leftBlack = s1Black; rightBlack = s2Black; }
}

bool anyBlack() {
  bool l, r;
  readLine(l, r);
  return l || r;          // detection = either sensor on black
}

bool dbBlack = false;
unsigned long dbSince = 0;
void resetDebounce() { dbBlack = false; }

// True once black has been seen continuously for LINE_DEBOUNCE_MS.
bool lineDetectedDebounced() {
  if (!anyBlack()) { dbBlack = false; return false; }
  if (!dbBlack) { dbBlack = true; dbSince = millis(); }
  return millis() - dbSince >= LINE_DEBOUNCE_MS;
}

// =====================================================================
//  BEHAVIOURS
// =====================================================================

// One step of right-wall nudge steering.
// targetCm < 0 means "no wall to hold": drive straight.
void wallStep(float targetCm, int speed) {
  float d = readDistance();
  if (targetCm < 0 || !isValidWall(d)) {
    setMotors(speed, speed);                       // null reading -> straight
  } else if (d < targetCm - WALL_TOLERANCE_CM) {
    setMotors(speed - NUDGE, speed);               // too close -> nudge left
  } else if (d > targetCm + WALL_TOLERANCE_CM) {
    setMotors(speed, speed - NUDGE);               // too far  -> nudge right
  } else {
    setMotors(speed, speed);                       // in band  -> straight
  }
}

// Read the wall now and use it as the target ("hold starting distance").
float holdTarget() {
  float d = readDistance();
  return isValidWall(d) ? d : -1;
}

// Wall-follow until a black line is detected.
// Lines are ignored for ignoreMs, then the robot must see white for
// LINE_CLEAR_MS (so the line it is standing on isn't counted).
// Returns false if aborted.
bool wallFollowUntilLine(unsigned long ignoreMs) {
  showState(F("WALL_FOLLOW"), 0, LED_BRIGHTNESS, 0);
  unsigned long t0 = millis();
  bool armed = false, whiteTiming = false;
  unsigned long whiteSince = 0;
  resetDebounce();

  while (true) {
    if (checkAbort()) return false;
    wallStep(WALL_TARGET_CM, BASE_SPEED);

    if (millis() - t0 < ignoreMs) continue;        // start-arrow ignore window

    if (!armed) {
      if (!anyBlack()) {
        if (!whiteTiming) { whiteTiming = true; whiteSince = millis(); }
        if (millis() - whiteSince >= LINE_CLEAR_MS) armed = true;
      } else {
        whiteTiming = false;
      }
      continue;
    }

    if (lineDetectedDebounced()) {
      brake();
      Serial.println(F("Line detected"));
      return true;
    }
  }
}

// Line #1: one continuous clockwise spin of SPIN_COUNT x 360°.
bool doSpins() {
  showState(F("SPIN_3x360_CW"), 0, 0, LED_BRIGHTNESS);
  unsigned long total = SPIN_TOTAL_OVERRIDE_MS > 0
                          ? SPIN_TOTAL_OVERRIDE_MS
                          : SPIN_360_MS * (unsigned long)SPIN_COUNT;
  return driveFor(SPIN_SPEED, -SPIN_SPEED, total);   // left fwd, right back = clockwise
}

// Forward a distance with nudging.
// Returns 1 = line found (only if stopOnLine), 0 = distance done, -1 = aborted.
int forwardNudged(float cm, float targetCm, bool stopOnLine) {
  unsigned long dur = (unsigned long)(cm * MS_PER_CM);
  unsigned long t0 = millis();
  resetDebounce();
  while (millis() - t0 < dur) {
    if (checkAbort()) return -1;
    if (stopOnLine && lineDetectedDebounced()) { brake(); return 1; }
    wallStep(targetCm, UT_SPEED);
  }
  brake();
  return 0;
}

// Line #2 U-turn. Returns 1 = crossbar found in F3, 0 = not found, -1 = aborted.
int doUTurn() {
  showState(F("U_TURN"), LED_BRIGHTNESS * 3 / 4, 0, LED_BRIGHTNESS * 3 / 4);
  if (forwardNudged(UT_FWD1_CM, WALL_TARGET_CM, false) < 0) return -1;  // F1: corridor A wall @15
  if (!pivotLeft90()) return -1;                                         // L1
  if (forwardNudged(UT_FWD2_CM, holdTarget(), false) < 0) return -1;     // F2: hold end-wall distance
  if (!pivotLeft90()) return -1;                                         // L2
  return forwardNudged(UT_FWD3_MAX_CM, holdTarget(), true);              // F3: until crossbar
}

// Creep across the crossbar.
// Returns 0 = crossed (white found), 1 = still black (already on the stem), -1 = aborted.
int crossCrossbar() {
  setMotors(CREEP_SPEED, CREEP_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < CROSSBAR_CLEAR_MS) {
    if (checkAbort()) return -1;
    if (!anyBlack()) { brake(); return 0; }
  }
  brake();
  return 1;
}

// ---- Sweep search ------------------------------------------------------------
unsigned long swStart, swWhiteSince;
bool swArmed, swWhiteTiming;

// Returns 1 = line found, 0 = keep going, -1 = aborted, -2 = timed out.
int sweepTick(int &foundSide) {
  if (checkAbort()) return -1;
  if (millis() - swStart >= SWEEP_TIMEOUT_MS) return -2;
  bool l, r;
  readLine(l, r);
  if (!swArmed) {
    if (!l && !r) {
      if (!swWhiteTiming) { swWhiteTiming = true; swWhiteSince = millis(); }
      if (millis() - swWhiteSince >= SWEEP_ARM_MS) swArmed = true;
    } else {
      swWhiteTiming = false;
    }
    return 0;
  }
  if (l || r) {
    if (l && !r) foundSide = SIDE_LEFT;
    else if (r && !l) foundSide = SIDE_RIGHT;
    return 1;
  }
  return 0;
}

int sweepMove(int left, int right, unsigned long ms, int &foundSide) {
  setMotors(left, right);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    int r = sweepTick(foundSide);
    if (r != 0) { brake(); return r; }
  }
  return 0;
}

// Zig-zag search with widening swings.
// Each cycle: swing back to centre, creep straight, swing out to the other side.
// Returns 1 = found, -1 = aborted, -2 = timed out.
int sweepSearch(int &foundSide) {
  showState(F("SWEEP_SEARCH"), LED_BRIGHTNESS, LED_BRIGHTNESS * 3 / 4, 0);
  swStart = millis();
  swArmed = false;
  swWhiteTiming = false;

  int dir = SWEEP_FIRST_DIR;
  unsigned long prevHalf = 0;
  unsigned long half = SWEEP_STEP_MS;

  while (true) {
    int outer = SWEEP_SPEED, inner = SWEEP_INNER_SPEED;
    int l = (dir == SIDE_RIGHT) ? outer : inner;
    int r = (dir == SIDE_RIGHT) ? inner : outer;
    foundSide = dir;
    int res;

    if (prevHalf > 0) {                                     // back to centre
      res = sweepMove(l, r, prevHalf, foundSide);
      if (res != 0) return res;
    }
    res = sweepMove(SWEEP_SPEED, SWEEP_SPEED, SWEEP_CREEP_MS, foundSide); // creep forward
    if (res != 0) return res;
    res = sweepMove(l, r, half, foundSide);                 // out to this side
    if (res != 0) return res;

    prevHalf = half;
    half = min(half + SWEEP_GROW_MS, SWEEP_MAX_MS);
    dir = -dir;
  }
}

// ---- Line following -------------------------------------------------------------
// Keeps both sensors on black. When lost, pivots toward the last side seen
// (for at most LF_SEARCH_MAX_MS), and finishes after LINE_END_MS without a line.
// Returns true = finished normally, false = aborted.
bool lineFollow(int lastSide) {
  showState(F("LINE_FOLLOW"), LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS);
  bool lost = false;
  unsigned long lostSince = 0;

  while (true) {
    if (checkAbort()) return false;
    bool l, r;
    readLine(l, r);

    if (l && r) {                                   // centred
      setMotors(LF_SPEED, LF_SPEED);
      lost = false;
    } else if (l) {                                 // line under left -> steer left
      setMotors(LF_INNER_SPEED, LF_SPEED);
      lastSide = SIDE_LEFT;
      lost = false;
    } else if (r) {                                 // line under right -> steer right
      setMotors(LF_SPEED, LF_INNER_SPEED);
      lastSide = SIDE_RIGHT;
      lost = false;
    } else {                                        // lost
      if (!lost) { lost = true; lostSince = millis(); }
      unsigned long lostFor = millis() - lostSince;
      if (lostFor >= LINE_END_MS) { brake(); return true; }
      if (lostFor < LF_SEARCH_MAX_MS) {
        if (lastSide == SIDE_LEFT) setMotors(-LF_SEARCH_SPEED, LF_SEARCH_SPEED);
        else                       setMotors(LF_SEARCH_SPEED, -LF_SEARCH_SPEED);
      } else {
        setMotors(0, 0);                            // searched far enough: wait it out
      }
    }
  }
}

// =====================================================================
//  END STATES
// =====================================================================
void finishCelebration() {
  Serial.println(F("FINISHED"));
  setMotors(0, 0);
  for (int i = 0; i < 3; i++) {
    setLed(0, LED_BRIGHTNESS, 0); beep(1568, 120);
    setLed(0, 0, 0);              delay(120);
  }
  setLed(0, LED_BRIGHTNESS, 0);
}

void failStop(const __FlashStringHelper *why) {
  setMotors(0, 0);
  Serial.print(F("STOPPED: "));
  Serial.println(why);
  setLed(LED_BRIGHTNESS, 0, 0);
  beep(300, 400);
}

// =====================================================================
//  COMPETITION RUN
//  startAt: 1 = from the start, 2 = after line #1, 3 = U-turn
// =====================================================================
void runCourse(int startAt) {
  if (startAt <= 1) {
    if (!wallFollowUntilLine(START_IGNORE_MS)) return;     // -> line #1
    if (!doSpins()) return;
  }
  if (startAt <= 2) {
    unsigned long ignore = (startAt == 2) ? START_IGNORE_MS : 0;
    if (!wallFollowUntilLine(ignore)) return;              // -> line #2
  }

  int ut = doUTurn();
  if (ut < 0) return;

  int side = SWEEP_FIRST_DIR;
  bool needSweep = true;

  if (ut == 1) {                                           // crossbar found in F3
    int c = crossCrossbar();
    if (c < 0) return;
    if (c == 1) {
      Serial.println(F("Already on the stem: skipping sweep"));
      needSweep = false;
    }
  } else {
    Serial.println(F("F3 ended without seeing the crossbar: sweeping anyway"));
  }

  if (needSweep) {
    int s = sweepSearch(side);
    if (s == -1) return;                                   // aborted
    if (s == -2) { failStop(F("sweep timed out")); return; }
  }

  if (!lineFollow(side)) return;
  finishCelebration();
}

// =====================================================================
//  CALIBRATION / TEST MODES
// =====================================================================
void calSpin() {
  Serial.println(F("CAL: spin test. Should end facing EXACTLY the start direction."));
  Serial.println(F("     Over-rotated -> lower SPIN_360_MS. Under-rotated -> raise it."));
  doSpins();
}

void calTurn90() {
  Serial.println(F("CAL: 4 x left 90. Should end facing the start direction."));
  Serial.println(F("     Over-rotated -> lower TURN90_MS. Under-rotated -> raise it."));
  showState(F("CAL_TURN90"), LED_BRIGHTNESS, LED_BRIGHTNESS / 3, 0);
  for (int i = 0; i < 4; i++) {
    if (!pivotLeft90()) return;
    delay(500);
  }
}

void calDistance() {
  Serial.print(F("CAL: driving "));
  Serial.print(CAL_DIST_CM);
  Serial.println(F(" cm. Measure the actual distance D (cm), then set"));
  Serial.println(F("     MS_PER_CM = MS_PER_CM * CAL_DIST_CM / D"));
  Serial.println(F("     (If it spins or reverses, fix INVERT_*_MOTOR.)"));
  showState(F("CAL_DISTANCE"), LED_BRIGHTNESS, LED_BRIGHTNESS / 3, 0);
  driveFor(UT_SPEED, UT_SPEED, (unsigned long)(CAL_DIST_CM * MS_PER_CM));
}

void calSensors() {
  Serial.println(F("CAL: sensor monitor (press button to stop)."));
  Serial.println(F("     Put tape under ONLY the robot's left sensor: should print L=1 R=0."));
  Serial.println(F("     If reversed, set SWAP_LINE_SENSORS = true."));
  setLed(LED_BRIGHTNESS, LED_BRIGHTNESS / 3, 0);
  while (!checkAbort()) {
    bool l, r;
    readLine(l, r);
    float d = readDistance();
    Serial.print(F("dist=")); Serial.print(d);
    Serial.print(isValidWall(d) ? F(" (wall)") : F(" (null)"));
    Serial.print(F("  L=")); Serial.print(l);
    Serial.print(F(" R=")); Serial.println(r);
    if (l && r)  setLed(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS);
    else if (l)  setLed(0, 0, LED_BRIGHTNESS);
    else if (r)  setLed(LED_BRIGHTNESS, LED_BRIGHTNESS, 0);
    else         setLed(0, 0, 0);
    delay(200);
  }
}

void calWallFollow() {
  Serial.println(F("CAL: wall-follow only (press button to stop)."));
  showState(F("CAL_WALL"), 0, LED_BRIGHTNESS, 0);
  while (!checkAbort()) wallStep(WALL_TARGET_CM, BASE_SPEED);
}

void calLineFollow() {
  Serial.println(F("CAL: line-follow only. Place on the line."));
  if (lineFollow(SWEEP_FIRST_DIR)) finishCelebration();
}

// =====================================================================
//  BUTTON SELECTION
// =====================================================================
int waitForPresses() {
  setLed(0, LED_BRIGHTNESS / 4, LED_BRIGHTNESS / 4);     // idle: dim cyan
  Serial.println(F("Ready: press button 1-9 times."));
  while (buttonDown()) {}                                // wait for release
  delay(50);

  int count = 0;
  unsigned long lastPress = 0;
  while (true) {
    if (buttonDown()) {
      delay(30);                                         // debounce
      if (buttonDown()) {
        count++;
        setLed(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS);
        beep(900, 50);
        while (buttonDown()) {}
        setLed(0, LED_BRIGHTNESS / 4, LED_BRIGHTNESS / 4);
        delay(30);
        lastPress = millis();
      }
    }
    if (count > 0 && millis() - lastPress >= START_DELAY_MS) return count;
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  led.setpin(13);                 // mBot onboard RGB LEDs
  setMotors(0, 0);
  setLed(0, 0, 0);
  beep(1000, 80);
}

void loop() {
  int mode = waitForPresses();
  Serial.print(F("Mode "));
  Serial.println(mode);
  aborted = false;

  switch (mode) {
    case 1: case 2: case 3: runCourse(mode); break;
    case 4: calSpin();       break;
    case 5: calTurn90();     break;
    case 6: calDistance();   break;
    case 7: calSensors();    aborted = false; break;   // button is the normal exit
    case 8: calWallFollow(); aborted = false; break;
    case 9: calLineFollow(); break;
    default:
      failStop(F("unknown mode"));
      delay(1000);
      return;
  }

  setMotors(0, 0);
  if (aborted) failStop(F("aborted"));
  delay(1500);                    // show the end colour before returning to idle
}
