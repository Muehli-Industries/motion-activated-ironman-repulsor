/*
  Iron Man Repulsor – Pose + Blast with NeoPixel Jewel
  -------------------------------------------------------------------------
  - Board: Seeed Studio XIAO ESP32-C3
  - Sensor: ADXL345 (I2C, address 0x53 typical)
  - LED: NeoPixel Jewel (7x WS2812B) on GPIO2 using FastLED
  - Audio: DFPlayer Mini on UART1 (GPIO4 RX<-DF TX, GPIO3 TX->DF RX)
  - Output: Serial debugging and DFPlayer sound playback

  Behavior:
    - When the repulsor pose is detected, the Jewel lights steady WHITE at max brightness.
    - While holding the pose, a fast wrist motion triggers a "Blast":
        * plays a sound
        * shows a short non-blocking white effect: the ENTIRE jewel strobes (all 7 LEDs)
    - Leaving the pose turns the Jewel off (all off) and plays a sound.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <FastLED.h>

// =====================================================================
// LED (FastLED / NeoPixel Jewel) – White-only rendering
// =====================================================================
#define DATA_PIN      2                 // GPIO2 → Jewel DIN
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB
#define NUM_LEDS      7

// Single global brightness (0..255) for both steady and blast
uint8_t BRIGHTNESS = 255;

CRGB leds[NUM_LEDS];

// --- Blast light effect (non-blocking) ---
// Global strobe: ALL 7 LEDs flicker synchronously (white ON / off)
unsigned long blastStartMs = 0;
bool          blastActive  = false;

// Tunables for blast timing/pattern
const unsigned long BLAST_TOTAL_MS      = 220; // total effect length
const unsigned long BLAST_STROBE_PERIOD = 40;  // on/off period (ms)
const uint8_t       BLAST_STROBE_DUTY   = 50;  // % time ON within each period

inline bool blastOnPhase(unsigned long now) {
  unsigned long t = now - blastStartMs;
  if (t >= BLAST_TOTAL_MS) return false;
  unsigned long phase = t % BLAST_STROBE_PERIOD;
  unsigned long onMs  = (BLAST_STROBE_PERIOD * BLAST_STROBE_DUTY) / 100;
  return phase < onMs;
}

// Render jewel: steady white if poseOn, global strobe during blast
inline void renderLeds(unsigned long now, bool poseOn) {
  if (!poseOn) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  } else {
    if (blastActive && (now - blastStartMs) < BLAST_TOTAL_MS) {
      bool on = blastOnPhase(now);
      fill_solid(leds, NUM_LEDS, on ? CRGB::White : CRGB::Black);
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::White);
    }
  }

  FastLED.setBrightness(BRIGHTNESS);
  FastLED.show();
}

// =====================================================================
// DFPlayer Mini (audio)
// =====================================================================
HardwareSerial dfSerial(1); // UART1
DFRobotDFPlayerMini dfplayer;
static const uint8_t DF_DEFAULT_VOL = 8; // 0..30

// DFPlayer wiring (raw GPIO)
constexpr int DFPLAYER_TX_TO_ESP_RX = 4; // DF TX → ESP RX
constexpr int DFPLAYER_RX_TO_ESP_TX = 3; // DF RX ← ESP TX

void Setup_DF_Player() {
  dfSerial.begin(9600, SERIAL_8N1, DFPLAYER_TX_TO_ESP_RX, DFPLAYER_RX_TO_ESP_TX);
  if (!dfplayer.begin(dfSerial)) {
    Serial.println("❌ DFPlayer not found!");
    return;
  }
  Serial.println("✅ DFPlayer ready.");
  dfplayer.volume(DF_DEFAULT_VOL);
}

// Play a sound (1 = 0001.mp3)
void PlaySound(uint16_t track = 1) {
  if (track == 0) track = 1;
  Serial.printf("Playing track %u\n", track);
  dfplayer.play(track);
}

// =====================================================================
// ADXL345 (pose detection)
// =====================================================================
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// -------------------- Pose thresholds --------------------
// Angles in degrees (Y–Z plane)
const float ON_ANGLE  = 55.0f; // enter pose at/above this
const float OFF_ANGLE = 40.0f; // leave pose at/under this
const float MAX_ENTER_ANGLE = 80.0f; // block entry near vertical

// Orientation gates
const float Y_MIN_ENTER       = 0.05f; // fingers up (+Y) to enter
const float Z_MIN_ENTER       = 0.10f; // back of hand up (+Z) to enter
const float Y_MIN_HOLD        = 0.02f; // relaxed hold
const float MAX_ROLL_X_ENTER  = 0.35f; // |nx| < ...
const float MAX_ROLL_X_HOLD   = 0.70f; // relaxed while holding

// Quasi-static gate for entering
const float G_TOL_ON = 0.25f;   // ||g|-1| < G_TOL_ON

// Debounce / timing
const unsigned long DEADTIME_MS    = 300; // re-entry deadtime after toggle
const unsigned long MIN_ON_MS      = 120; // minimal ON time before allowing OFF
const unsigned long OFF_CONFIRM_MS = 150; // must leave pose this long

// Smoothing (EMA) for gravity vector
const float ALPHA = 0.2f;

// -------------------- BLAST detection --------------------
#define BACKWARD_JERK_IS_POS_Y 0   // 0 = expect jerk toward -Y for recoil
const float BLAST_MIN_ANGLE       = 55.0f;   // must be a high-angle pose
const unsigned long BLAST_COOLDOWN_MS = 500; // min time between blasts
const unsigned long BLAST_ARM_MS      = 150; // arming delay after LED ON
const unsigned long BLAST_MIN_ON_BEFORE_FIRE_MS = 1000; // must hold ON ≥1s

// Post-blast protections
const unsigned long POST_BLAST_GRACE_MS = 350;  // ignore OFF for this long
const unsigned long POST_BLAST_MIN_ON_MS = 250; // ensure ON at least this long after blast

// Calm window (for arming)
const float          CALM_LMAG_THRESH  = 0.25f;
const unsigned long  CALM_REQUIRED_MS  = 60;

// Path A: Angle snap (windowed delta)
const float ANGLE_WINDOW_MS     = 120.0f;
const float ANGLE_DELTA_MIN     = 12.0f;
const float MIN_LMAG_FOR_ANGLE  = 0.15f;
const unsigned long ANGLE_COOLDOWN_MS = 1500;

// Path B: Directional Y-jerk
const float BLAST_Y_THRESH     = 0.20f;
const float BLAST_TOTAL_THRESH = 0.28f;
const float BLAST_Y_DOM        = 0.55f;

// -------------------- State --------------------
bool ledOn = false;                 // logical pose state → we render Jewel from this
unsigned long lastOnTs = 0;
unsigned long lastToggle = 0;
unsigned long offCandidateSince = 0;

unsigned long lastBlastTs = 0;       // common blast cooldown
unsigned long lastAngleBlastTs = 0;  // extra cooldown for angle path
unsigned long calmSince = 0;         // 0 = not calm yet

// Smoothed gravity direction
float nx_f = 0.0f, ny_f = 0.0f, nz_f = 1.0f;

// Angle helpers
float anglePrev = 0.0f;
unsigned long anglePrevTs = 0;

// Windowed angle baseline
float angleWinRef = 0.0f;
unsigned long angleWinTs = 0;

// Blast arming
bool angleArmed = false;

// -------------------- Debug switch --------------------
// #define VERBOSE_DEBUG 1

// =====================================================================
// Setup
// =====================================================================
void setup() {
  // LED / FastLED init
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  Serial.begin(115200);
  delay(50);

  Setup_DF_Player();
  Serial.println("Repulsor controller starting...");

  Wire.begin();
  if (!accel.begin()) {
    Serial.println("ERROR: ADXL345 not detected (address 0x53?). Check wiring.");
    // Show solid red to indicate sensor error
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
    while (1) delay(10);
  }
  accel.setRange(ADXL345_RANGE_2_G);
  Serial.println("ADXL345 initialized (±2g).");
}

// =====================================================================
// Loop
// =====================================================================
void loop() {
  sensors_event_t e;
  accel.getEvent(&e); // m/s^2 (gravity + motion)

  // Convert to g-units
  const float inv_g = 1.0f / 9.80665f;
  float gx = e.acceleration.x * inv_g;
  float gy = e.acceleration.y * inv_g;
  float gz = e.acceleration.z * inv_g;

  // Gravity magnitude and normalized components
  float gnorm = sqrtf(gx*gx + gy*gy + gz*gz);
  if (gnorm < 1e-3f) return;

  float nx = gx / gnorm, ny = gy / gnorm, nz = gz / gnorm;

  // EMA smoothing
  nx_f = ALPHA*nx_f + (1-ALPHA)*nx;
  ny_f = ALPHA*ny_f + (1-ALPHA)*ny;
  nz_f = ALPHA*nz_f + (1-ALPHA)*nz;

  // Pose gates
  bool fingerUp_enter  = (ny_f > Y_MIN_ENTER);
  bool backUp_enter    = (nz_f > Z_MIN_ENTER);
  bool rollSmall_enter = (fabsf(nx_f) < MAX_ROLL_X_ENTER);
  bool quasiStatic     = (fabsf(gnorm - 1.0f) < G_TOL_ON);
  bool allowEnter      = fingerUp_enter && backUp_enter && rollSmall_enter && quasiStatic;

  bool fingerUp_hold   = (ny_f > Y_MIN_HOLD);
  bool rollSmall_hold  = (fabsf(nx_f) < MAX_ROLL_X_HOLD);
  bool allowHold       = fingerUp_hold && rollSmall_hold; // Z not required while holding

  // Angle in Y–Z plane (0..90°) using positive parts
  float y = (ny_f > 0) ? ny_f : 0.0f;
  float z = (nz_f > 0) ? nz_f : 0.0f;
  if (z < 1e-3f) z = 1e-3f;
  float angleYZ = atan2f(y, z) * 180.0f / PI;

  unsigned long now = millis();
  bool deadtimeOver = (now - lastToggle) > DEADTIME_MS;

  // -------------------- LED ON (strict entry) --------------------
  if (!ledOn && deadtimeOver && allowEnter &&
      angleYZ >= ON_ANGLE && angleYZ <= MAX_ENTER_ANGLE) {
    ledOn = true;
    PlaySound(2); // power on
    lastToggle = now;
    lastOnTs   = now;
    offCandidateSince = 0;
    angleArmed = false; // will arm only after calm (below)
    Serial.printf("LED ON (angle exceeded)  angle=%.1f  nx=%.2f ny=%.2f nz=%.2f  |g|=%.2f\n",
                  angleYZ, nx_f, ny_f, nz_f, gnorm);
  }

  // -------------------- LED OFF (leave pose) --------------------
  bool leavePose    = (!allowHold) || (angleYZ <= OFF_ANGLE);
  bool minOnElapsed = (now - lastOnTs) >= MIN_ON_MS;

  // suppress OFF shortly after a blast (grace window)
  bool inBlastGrace = (now - lastBlastTs) < POST_BLAST_GRACE_MS;
  if (inBlastGrace) {
    leavePose = false;
  }

  if (ledOn && leavePose && minOnElapsed) {
    if (offCandidateSince == 0) offCandidateSince = now;
    if ((now - offCandidateSince) >= OFF_CONFIRM_MS) {
      ledOn = false;
      PlaySound(1); // power off
      offCandidateSince = 0;
      angleArmed = false;
      Serial.printf("LED OFF (pose left)       angle=%.1f  nx=%.2f ny=%.2f nz=%.2f  |g|=%.2f\n",
                    angleYZ, nx_f, ny_f, nz_f, gnorm);
    }
  } else {
    offCandidateSince = 0;
  }

  // -------------------- Linear acceleration (calm indicator) ---
  float lmag_simple = fabsf(gnorm - 1.0f);  // simple proxy for linear accel

  // Calm tracking (for ARming)
  if (lmag_simple < CALM_LMAG_THRESH) {
    if (calmSince == 0) calmSince = now;
  } else {
    calmSince = 0;
  }

  // -------------------- Angle rate (info) ----------------------
  float angleRate = 0.0f;
  if (anglePrevTs != 0) {
    float dt = (now - anglePrevTs) / 1000.0f;
    if (dt > 1e-3f) angleRate = (angleYZ - anglePrev) / dt;
  }
  anglePrev = angleYZ;
  anglePrevTs = now;

  // -------------------- Windowed delta -------------------------
  if (angleWinTs == 0) { angleWinTs = now; angleWinRef = angleYZ; }
  unsigned long winAge = now - angleWinTs;
  float deltaAngle = angleYZ - angleWinRef;
  if (winAge > ANGLE_WINDOW_MS) {
    angleWinRef = angleYZ;
    angleWinTs  = now;
    deltaAngle  = 0.0f;
  }

  // -------------------- Safety: disarm if LED off or pose lost ---
  if (!ledOn || !allowHold) {
#ifdef VERBOSE_DEBUG
    if (angleArmed) Serial.println("Angle-snap DISARMED (LED off or pose left)");
#endif
    angleArmed = false;
  }

  // -------------------- Angle-snap ARming (calm at top) --------
  if (!angleArmed) {
    bool calmReady = (calmSince != 0) && ((now - calmSince) >= CALM_REQUIRED_MS);
    if (ledOn && allowHold && calmReady && (now - lastOnTs) >= BLAST_ARM_MS) {
      angleArmed  = true;
      angleWinRef = angleYZ; // baseline at arming
      angleWinTs  = now;
#ifdef VERBOSE_DEBUG
      Serial.printf("Angle-snap ARMED at angle=%.1f (calm)\n", angleYZ);
#endif
    }
  }

  // -------------------- BLAST conditions (only if LED ON) ------
  if (ledOn) {
    // ---- Path A: Angle snap ----
    bool angleBlastOK =
      ledOn &&
      angleArmed &&
      allowHold &&
      (angleYZ >= BLAST_MIN_ANGLE) &&
      (now - lastBlastTs)      >= BLAST_COOLDOWN_MS &&
      (now - lastAngleBlastTs) >= ANGLE_COOLDOWN_MS &&
      (now - lastOnTs)         >= BLAST_MIN_ON_BEFORE_FIRE_MS &&
      (angleYZ > angleWinRef + 0.5f) &&
      (deltaAngle >= ANGLE_DELTA_MIN) &&
      (lmag_simple >= MIN_LMAG_FOR_ANGLE);

    if (angleBlastOK) {
      Serial.println("BLAST! (angle-rate)");
      PlaySound(3);
      lastBlastTs = now;
      lastAngleBlastTs = now;
      angleArmed = false;
      calmSince  = 0;
      angleWinRef = angleYZ;
      angleWinTs  = now;

      // start light effect (global strobe)
      blastActive  = true;
      blastStartMs = now;

      // ensure LED stays on briefly after blast
      if ((now - lastOnTs) < POST_BLAST_MIN_ON_MS) {
        lastOnTs = now - POST_BLAST_MIN_ON_MS;
      }
    }

    // ---- Path B: Directional Y-jerk ----
    float lx = gx - nx_f, ly = gy - ny_f, lz = gz - nz_f;
    float lmag_est  = sqrtf(lx*lx + ly*ly + lz*lz);

#if BACKWARD_JERK_IS_POS_Y
    bool backwardJerk = (ly) > BLAST_Y_THRESH;   // +Y jerk
#else
    bool backwardJerk = (-ly) > BLAST_Y_THRESH;  // -Y jerk
#endif
    bool strongEnough = (lmag_est > BLAST_TOTAL_THRESH);
    bool yDominant    = (lmag_est > 1e-3f) ? (fabsf(ly) / lmag_est >= BLAST_Y_DOM) : false;
    bool inPoseBlast  = (angleYZ >= BLAST_MIN_ANGLE) && allowHold;
    bool preCalmOK    = (calmSince != 0) && ((now - calmSince) >= CALM_REQUIRED_MS);

    bool jerkBlastOK =
      ledOn &&
      (now - lastBlastTs) >= BLAST_COOLDOWN_MS &&
      (now - lastOnTs)    >= BLAST_MIN_ON_BEFORE_FIRE_MS &&
      backwardJerk && strongEnough && yDominant &&
      inPoseBlast && preCalmOK;

    if (jerkBlastOK) {
      Serial.println("BLAST! (jerk)");
      PlaySound(3);
      lastBlastTs = now;
      calmSince = 0; // reset calm

      // start light effect (global strobe)
      blastActive  = true;
      blastStartMs = now;

      // ensure LED stays on briefly after blast
      if ((now - lastOnTs) < POST_BLAST_MIN_ON_MS) {
        lastOnTs = now - POST_BLAST_MIN_ON_MS;
      }
      // angleArmed unchanged (independent path)
    }
  } // end if (ledOn)

#ifdef VERBOSE_DEBUG
  // Optional debug snapshot
  float lx_dbg = gx - nx_f, ly_dbg = gy - ny_f, lz_dbg = gz - nz_f;
  float lmag_est_dbg  = sqrtf(lx_dbg*lx_dbg + ly_dbg*ly_dbg + lz_dbg*lz_dbg);

#if BACKWARD_JERK_IS_POS_Y
  bool backwardJerk_dbg = (ly_dbg) > BLAST_Y_THRESH;
#else
  bool backwardJerk_dbg = (-ly_dbg) > BLAST_Y_THRESH;
#endif
  bool yDominant_dbg  = (lmag_est_dbg > 1e-3f) ? (fabsf(ly_dbg) / lmag_est_dbg >= BLAST_Y_DOM) : false;
  bool inPose_dbg     = (angleYZ >= BLAST_MIN_ANGLE) && allowHold;

  bool Aok_dbg =
    ledOn && angleArmed && allowHold &&
    (angleYZ >= BLAST_MIN_ANGLE) &&
    (now - lastBlastTs)      >= BLAST_COOLDOWN_MS &&
    (now - lastAngleBlastTs) >= ANGLE_COOLDOWN_MS &&
    (now - lastOnTs)         >= BLAST_MIN_ON_BEFORE_FIRE_MS &&
    (angleYZ > angleWinRef + 0.5f) &&
    (deltaAngle >= ANGLE_DELTA_MIN) &&
    (fabsf(gnorm - 1.0f) >= MIN_LMAG_FOR_ANGLE);

  bool Jok_dbg =
    ledOn &&
    (now - lastBlastTs) >= BLAST_COOLDOWN_MS &&
    (now - lastOnTs)    >= BLAST_MIN_ON_BEFORE_FIRE_MS &&
    backwardJerk_dbg && (lmag_est_dbg > BLAST_TOTAL_THRESH) &&
    yDominant_dbg && inPose_dbg &&
    ((calmSince != 0) && ((now - calmSince) >= CALM_REQUIRED_MS));

  Serial.printf(
    "Chk angle=%.1f dA=%.1f rate=%.0f lmag=%.2f armed=%d Aok=%d Jok=%d inPose=%d calm=%d blastAct=%d\n",
    angleYZ, deltaAngle, angleRate, fabsf(gnorm - 1.0f),
    angleArmed ? 1 : 0, Aok_dbg ? 1 : 0, Jok_dbg ? 1 : 0,
    inPose_dbg ? 1 : 0,
    (calmSince && (now - calmSince) >= CALM_REQUIRED_MS) ? 1 : 0,
    blastActive ? 1 : 0
  );
#endif

  // Stop blast after window ends
  if (blastActive && (now - blastStartMs) >= BLAST_TOTAL_MS) {
    blastActive = false;
  }

  // Final: render LEDs each loop
  renderLeds(now, ledOn);

  delay(10); // ~100 Hz loop
}