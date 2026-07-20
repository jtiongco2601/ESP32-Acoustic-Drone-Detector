// ============================================================
// ACOUSTIC DRONE DETECTION MVP
// ESP32 + 2x INMP441 (shared I2S bus, L/R slots) + SSD1306 OLED
// + vibration motor via PN2222 on GPIO 27
//
// Honest capability statement:
// Passive acoustic warning device. Two microphones give rough
// LEFT / CENTER / RIGHT side bias only, no bearing or range.
// A fan, motor, radio or power tool can share parts of a drone
// spectrum; the multi-feature score reduces but cannot remove
// that overlap. Real validation requires actual drone flights.
//
// Pins (fixed, do not change):
//   I2S_WS  GPIO 25 | I2S_SCK GPIO 26 | I2S_SD GPIO 33
//   Motor   GPIO 27 | OLED SDA GPIO 21 | OLED SCL GPIO 22
//
// Requires: arduinoFFT >= 2.0.0 (v2 API), Adafruit SSD1306 + GFX
// ============================================================

#include <Arduino.h>
#include <driver/i2s.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>
#include <math.h>

// ============================================================
// 1. MAIN SWITCHES
// ============================================================

#define DEBUG_MODE            true      // Serial diagnostics on/off
#define SERIAL_PLOTTER_MODE   true      // true: numeric plotter output
#define POWER_SOURCE_POWERBANK true     // true: show USB tag on OLED

// If field testing shows LEFT/RIGHT are reversed, flip this.
// The classic ESP32 I2S peripheral does not guarantee which
// physical mic (L/R pin low vs high) lands in the even slot,
// so this MUST be verified once by tapping each mic. See docs.
#define SWAP_MIC_CHANNELS     false

// ============================================================
// 2. AUDIO CONFIGURATION
// ============================================================

#define SAMPLE_RATE       16000
#define FFT_SIZE          1024                    // 64 ms window
#define HOP_SIZE          512                     // 50% overlap, 32 ms hop
static const float FREQ_RES = (float)SAMPLE_RATE / (float)FFT_SIZE; // 15.625 Hz

// ============================================================
// 3. CALIBRATION AND THRESHOLDS (tune these first)
// ============================================================

#define CALIBRATION_MS        5000UL   // startup listen time
#define CAL_SETTLE_MS          500UL   // ignore startup/I2S settling transient
#define M_ABSOLUTE_FLOOR      1500.0f  // M never drops below this (spec)
#define CAL_STD_MULT          2.0f     // use measured variation without overreacting
#define CAL_MARGIN_MULT       0.25f    // minimum margin above background
#define CAL_MAX_MARGIN_MULT   0.50f    // cap spike-driven calibration margin
#define CAL_MARGIN_OFFSET     100.0f   // fixed margin over background
#define BG_ADAPT_UP_ALPHA     0.001f   // learn rising background very slowly
#define BG_ADAPT_DOWN_ALPHA   0.020f   // recover quickly from a high calibration
#define BG_ADAPT_MAX_RATIO    2.0f     // ignore E > 2*M when adapting bg

// Research-informed acoustic search bands. Published quadcopter measurements
// show shaft-rate/BPF tones plus sustained harmonics, sometimes extending to
// about 6 kHz. There is no universal "drone Hz", so use a harmonic family.
#define FUND_MIN_HZ           70.0f
#define FUND_MAX_HZ           800.0f
#define BAND_MAX_HZ           6000.0f  // below 8 kHz Nyquist at 16 kHz sample rate
#define MAX_HARMONIC          6

// Feature gates
#define FUND_MIN_SNR          4.0f     // peak must be 4x spectral noise floor
#define HARMONIC_MIN_RATIO    0.15f    // harmonic mag vs fundamental mag
#define HARMONIC_MIN_FLOOR    3.0f     // harmonic mag vs noise floor
#define HARMONIC_TOL_FRAC     0.04f    // +/-4% tolerance (RPM drift)
#define TONE_OUTSIDE_MIN      0.30f    // below this = pure sine -> reject
#define TONE_OUTSIDE_GOOD     0.55f    // at/above this = full spread points
#define NOISE_OUTSIDE_MAX     0.97f    // above this = shapeless noise
#define MIC_AGREE_MIN         0.25f    // quiet/loud mic ratio floor
#define MIC_AGREE_GOOD        0.50f    // full agreement points at this ratio
#define MOD_FSTD_MIN_HZ       0.8f     // below: too static (pure tone)
#define MOD_FSTD_FULL_HZ      2.5f     // full modulation points from here
#define MOD_FSTD_TAPER_HZ     45.0f    // above: start penalizing (hopping)
#define MOD_FSTD_ZERO_HZ      80.0f    // at/above: no modulation points

// Hard evidence gates. These must pass together before persistence grows.
#define REQUIRED_HARMONICS    3
#define DIST_EVIDENCE_MIN     8.0f
#define MOD_EVIDENCE_MIN      5.0f
#define MIC_EVIDENCE_MIN      5.0f

// Persistence
#define PERSIST_TARGET_WIN    94       // ~3 s of good windows for full points
#define PERSIST_GOOD_GAIN     2        // reward consistent evidence
#define PERSIST_BAD_DECAY     1        // tolerate playback/propeller variation
#define EVIDENCE_HISTORY_WIN  64       // ~2.05 s at one 32 ms hop
#define EVIDENCE_MIN_FILLED   32       // require ~1 s before duty is trusted
#define EVIDENCE_DUTY_MIN     0.65f    // at least 65% strong windows
#define EVIDENCE_CONF         50.0f    // smoothed C at/above this = evidence
#define EVIDENCE_ABSENT_MS    2000UL   // evidence gone this long -> NO_DRONE

// State machine timing (normal mode)
#define CHECKING_MIN_MS       500UL
#define PROBABLE_MIN_MS       2500UL
#define DRONE_MIN_MS          5000UL
#define CONF_CHECKING         50.0f
#define CONF_PROBABLE         70.0f
#define CONF_DRONE            85.0f
#define DOWNGRADE_MS          1500UL   // PROBABLE/DRONE fall-back grace
#define DRONE_GRADE_HOLD_MS   2000UL   // C>=85 + hard gates continuously
#define DRONE_RELEASE_MS      1000UL   // release quickly when hard gates fail

// Confidence smoothing (per 32 ms hop)
#define CONF_SMOOTH_ALPHA     0.25f

// Direction
#define DIR_BIAS_LIMIT        0.12f    // |bias| above this = LEFT/RIGHT
#define DIR_SMOOTH_ALPHA      0.04f    // ~1 s smoothing at 31 hops/s
#define DIR_MIN_ENERGY_FRAC   0.7f     // update bias only when E > 0.7*M

// Motor pulse pattern while an alarm state is active
#define MOTOR_PULSE_ON_MS     400UL
#define MOTOR_PULSE_OFF_MS    400UL

// UI / diagnostics rates
#define OLED_UPDATE_MS        150UL
#define DEBUG_PRINT_MS        500UL
#define PLOTTER_PRINT_MS      100UL

// ============================================================
// PINS (fixed)
// ============================================================

#define I2S_WS        25
#define I2S_SCK       26
#define I2S_SD        33
#define I2S_PORT      I2S_NUM_0
#define VIBRATION_PIN 27
#define OLED_SDA      21
#define OLED_SCL      22
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================
// STATE
// ============================================================

enum DetectorState : uint8_t {
  ST_CALIBRATING,
  ST_NO_DRONE,
  ST_CHECKING,
  ST_PROBABLE,
  ST_DRONE
};

struct FeatureScores {
  float snr;         // 0..15
  float harmonics;   // 0..25
  float distribution;// 0..15
  float modulation;  // 0..15
  float persistence; // 0..20
  float micAgree;    // 0..10
  float total;       // 0..100 (clamped)
};

// Audio buffers (static, off the stack)
static int32_t i2sBuf[HOP_SIZE * 2];         // interleaved stereo, one hop
static int32_t lTmp[HOP_SIZE];
static int32_t rTmp[HOP_SIZE];
static float   leftFftBuf[FFT_SIZE];         // rolling left window
static float   rightFftBuf[FFT_SIZE];        // rolling right window
static float   vReal[FFT_SIZE];
static float   vImag[FFT_SIZE];
static float   leftMagnitude[FFT_SIZE / 2 + 1];

ArduinoFFT<float> FFT(vReal, vImag, (uint_fast16_t)FFT_SIZE,
                      (float)SAMPLE_RATE, true);

// Energies (same >>8 scale family as the original code)
static float leftEnergy = 0.0f;
static float rightEnergy = 0.0f;
static float combinedEnergy = 0.0f;

// Calibration / background
static float bgEnergy = 0.0f;        // background energy estimate
static float thresholdM = M_ABSOLUTE_FLOOR;
static double calSum = 0.0, calSumSq = 0.0;
static uint32_t calCount = 0;
static uint32_t calStartMs = 0;

// Spectrum results (per hop)
static float noiseFloorMag = 1.0f;
static float livePeakFreq = 0.0f;    // strongest live peak, for OLED only
static float fundFreq = 0.0f;        // 0 = no valid fundamental
static float fundMag = 0.0f;
static int   harmonicsFound = 0;
static int   harmonicsPossible = 0;
static float harmonicCoherence = 0.0f;
static float outsideRatio = 0.0f;    // energy share outside fundamental bins

// Modulation tracking (EMA mean/variance, reset per candidate)
static float fMean = 0.0f, fVar = 0.0f;
static float aMean = 0.0f, aVar = 0.0f;
static uint32_t modSamples = 0;

// Persistence / candidate tracking
static int      evidenceWindows = 0;
static uint16_t unstableWindows = 0;   // above M but failing tests
static uint16_t badWindows = 0;        // below M during a candidate
static bool     candidateActive = false;
static uint32_t candidateStartMs = 0;
static bool     strongWindowEvidence = false;
static uint8_t  evidenceHistory[EVIDENCE_HISTORY_WIN] = {0};
static uint16_t evidenceHistoryIndex = 0;
static uint16_t evidenceHistoryCount = 0;
static uint16_t evidenceHistoryGood = 0;
static float    evidenceDuty = 0.0f;
static bool     dutyEvidenceOk = false;

// Confidence
static FeatureScores scores = {0};
static float confSmoothed = 0.0f;

// State machine
static DetectorState state = ST_CALIBRATING;
static uint32_t stateEnteredMs = 0;
static uint32_t lastEvidenceMs = 0;
static uint32_t belowGradeMs = 0;      // for PROBABLE/DRONE downgrade
static uint32_t droneGradeSinceMs = 0;

// Direction
static float dirBias = 0.0f;

// UI / debug pacing
static uint32_t lastOledMs = 0;
static uint32_t lastDebugMs = 0;

// Motor
static bool motorOn = false;
static uint32_t motorPhaseMs = 0;

// ============================================================
// SETUP: I2S (config preserved from the original working code)
// ============================================================

bool setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // both mic slots
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 128,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.print("I2S driver install failed: ");
    Serial.println((int)err);
    return false;
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.print("I2S pin setup failed: ");
    Serial.println((int)err);
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }

  err = i2s_zero_dma_buffer(I2S_PORT);
  if (err != ESP_OK) {
    Serial.print("I2S DMA clear failed: ");
    Serial.println((int)err);
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  return true;
}

void haltWithError(const char* line1, const char* line2) {
  digitalWrite(VIBRATION_PIN, LOW);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.println(line1);
  display.setCursor(0, 24);
  display.println(line2);
  display.display();
  Serial.print(line1);
  Serial.print(": ");
  Serial.println(line2);
  while (true) { delay(1000); }
}

// ============================================================
// SETUP: OLED (address 0x3C and init preserved)
// ============================================================

void setupOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);   // faster I2C so display writes stay short

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // Display is the primary UI; without it, halt visibly on Serial.
    Serial.println("OLED init failed (0x3C). Halting.");
    while (true) { delay(1000); }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Drone Detector MVP");
  display.setTextSize(2);
  display.setCursor(0, 22);
  display.println("BOOT");
  display.display();
}

// ============================================================
// AUDIO: read one hop (512 stereo frames = 32 ms), DC-remove,
// compute per-channel energies, append both channels to rolling windows
// ============================================================

void readAudioHop() {
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, i2sBuf, sizeof(i2sBuf), &bytesRead, portMAX_DELAY);

  int frames = (int)(bytesRead / (2 * sizeof(int32_t)));
  if (frames <= 0) {                       // should not happen; stay safe
    leftEnergy = rightEnergy = combinedEnergy = 0.0f;
    return;
  }
  if (frames > HOP_SIZE) frames = HOP_SIZE;

  // Pass 1: extract channels (>>8 keeps the 24-bit INMP441 payload,
  // arithmetic shift preserves sign) and accumulate DC means.
  int64_t lSum = 0, rSum = 0;
  for (int i = 0; i < frames; i++) {
#if SWAP_MIC_CHANNELS
    int32_t l = i2sBuf[2 * i + 1] >> 8;
    int32_t r = i2sBuf[2 * i]     >> 8;
#else
    int32_t l = i2sBuf[2 * i]     >> 8;
    int32_t r = i2sBuf[2 * i + 1] >> 8;
#endif
    lTmp[i] = l;
    rTmp[i] = r;
    lSum += l;
    rSum += r;
  }
  const float lMean = (float)lSum / (float)frames;
  const float rMean = (float)rSum / (float)frames;

  // Pass 2: DC-removed energies and separate FFT-window fill.
  // Keeping the channels separate prevents phase cancellation when the
  // same sound reaches the two spaced microphones at different phases.
  memmove(leftFftBuf, leftFftBuf + HOP_SIZE,
          (FFT_SIZE - HOP_SIZE) * sizeof(float));
  memmove(rightFftBuf, rightFftBuf + HOP_SIZE,
          (FFT_SIZE - HOP_SIZE) * sizeof(float));

  double lAbs = 0.0, rAbs = 0.0;
  for (int i = 0; i < frames; i++) {
    const float lf = (float)lTmp[i] - lMean;
    const float rf = (float)rTmp[i] - rMean;
    lAbs += fabsf(lf);
    rAbs += fabsf(rf);
    leftFftBuf[FFT_SIZE - HOP_SIZE + i] = lf;
    rightFftBuf[FFT_SIZE - HOP_SIZE + i] = rf;
  }
  // If the driver ever returns a short read, pad with silence.
  for (int i = frames; i < HOP_SIZE; i++) {
    leftFftBuf[FFT_SIZE - HOP_SIZE + i] = 0.0f;
    rightFftBuf[FFT_SIZE - HOP_SIZE + i] = 0.0f;
  }

  leftEnergy  = (float)(lAbs / frames);
  rightEnergy = (float)(rAbs / frames);
  combinedEnergy = 0.5f * (leftEnergy + rightEnergy);
}

// ============================================================
// FFT on both 1024-sample windows. The per-bin maximum magnitude is
// used for detection, avoiding time-domain cancellation between mics.
// ============================================================

void runFFT() {
  // Left channel
  memcpy(vReal, leftFftBuf, sizeof(vReal));
  memset(vImag, 0, sizeof(vImag));
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  memcpy(leftMagnitude, vReal, sizeof(leftMagnitude));

  // Right channel
  memcpy(vReal, rightFftBuf, sizeof(vReal));
  memset(vImag, 0, sizeof(vImag));
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Preserve the stronger spectral evidence from either microphone.
  for (int b = 0; b <= FFT_SIZE / 2; b++) {
    if (leftMagnitude[b] > vReal[b]) vReal[b] = leftMagnitude[b];
  }
}

static inline int binOf(float hz) {
  int b = (int)lroundf(hz / FREQ_RES);
  if (b < 1) b = 1;
  if (b > FFT_SIZE / 2 - 1) b = FFT_SIZE / 2 - 1;
  return b;
}

// ============================================================
// SPECTRUM ANALYSIS: noise floor, shaft/BPF candidate, harmonic
// family and spectral spread. Dataset lesson: the strongest low-frequency
// peak is not always the true base, so candidates are ranked as a comb.
// ============================================================

float localPeakMagnitude(int center, int tolerance, int loLimit, int hiLimit) {
  int lo = center - tolerance;
  int hi = center + tolerance;
  if (lo < loLimit) lo = loLimit;
  if (hi > hiLimit) hi = hiLimit;
  float peak = 0.0f;
  for (int b = lo; b <= hi; b++) {
    if (vReal[b] > peak) peak = vReal[b];
  }
  return peak;
}

void analyzeSpectrum() {
  const int bandLo = binOf(FUND_MIN_HZ);          // ~bin 5, skips DC/handling
  const int fundHi = binOf(FUND_MAX_HZ);          // ~bin 32
  const int bandHi = binOf(BAND_MAX_HZ);          // ~bin 256

  // Live frequency reader for the OLED. This is deliberately separate from
  // drone scoring: it simply reports the strongest current peak from about
  // 50 Hz to 6 kHz, including ordinary background sounds.
  const int liveLo = binOf(50.0f);
  int livePeakBin = liveLo;
  float livePeakMag = 0.0f;
  for (int b = liveLo; b <= bandHi; b++) {
    if (vReal[b] > livePeakMag) {
      livePeakMag = vReal[b];
      livePeakBin = b;
    }
  }
  livePeakFreq = (float)livePeakBin * FREQ_RES;

  // --- Noise floor: mean of the band, then mean of bins below
  //     2x that mean, so strong peaks don't inflate the floor.
  float sum = 0.0f;
  for (int b = bandLo; b <= bandHi; b++) sum += vReal[b];
  const int bandCount = bandHi - bandLo + 1;
  const float rawAvg = sum / (float)bandCount;

  float lowSum = 0.0f;
  int lowCount = 0;
  for (int b = bandLo; b <= bandHi; b++) {
    if (vReal[b] < 2.0f * rawAvg) { lowSum += vReal[b]; lowCount++; }
  }
  noiseFloorMag = (lowCount > 0) ? (lowSum / (float)lowCount) : rawAvg;
  if (noiseFloorMag < 1e-6f) noiseFloorMag = 1e-6f;

  // --- Search 70..800 Hz for the best harmonic comb. Ranking the entire
  // family avoids assuming that the fundamental is the loudest component.
  int peakBin = bandLo;
  float peakMag = 0.0f;
  float bestComb = -1.0f;
  for (int candidate = bandLo; candidate <= fundHi; candidate++) {
    const float baseMag = vReal[candidate];
    if (baseMag <= FUND_MIN_SNR * noiseFloorMag) continue;

    const float baseHz = (float)candidate * FREQ_RES;
    int matches = 0;
    float baseSnr = baseMag / noiseFloorMag;
    if (baseSnr > 12.0f) baseSnr = 12.0f;
    float comb = baseSnr;
    for (int h = 2; h <= MAX_HARMONIC; h++) {
      const float target = baseHz * (float)h;
      if (target > BAND_MAX_HZ) break;
      int tol = (int)ceilf((target * HARMONIC_TOL_FRAC) / FREQ_RES);
      if (tol < 2) tol = 2;
      const float hMag = localPeakMagnitude(binOf(target), tol, bandLo, bandHi);
      if (hMag > HARMONIC_MIN_RATIO * baseMag &&
          hMag > HARMONIC_MIN_FLOOR * noiseFloorMag) {
        matches++;
        float relativeMag = hMag / baseMag;
        if (relativeMag > 2.0f) relativeMag = 2.0f;
        comb += 12.0f + relativeMag;
      }
    }
    if (comb > bestComb) {
      bestComb = comb;
      peakBin = candidate;
      peakMag = baseMag;
    }
  }

  fundFreq = 0.0f;
  fundMag = peakMag;
  if (peakMag > FUND_MIN_SNR * noiseFloorMag) {
    // Parabolic interpolation for sub-bin frequency (matters because
    // one bin is 15.6 Hz wide and the modulation feature needs finer
    // resolution than that).
    const float mL = vReal[peakBin - 1];
    const float mC = vReal[peakBin];
    const float mR = vReal[peakBin + 1];
    const float denom = mL - 2.0f * mC + mR;
    float delta = 0.0f;
    if (fabsf(denom) > 1e-9f) {
      delta = 0.5f * (mL - mR) / denom;
      if (delta > 0.5f) delta = 0.5f;
      if (delta < -0.5f) delta = -0.5f;
    }
    fundFreq = ((float)peakBin + delta) * FREQ_RES;
  }

  // --- Recount harmonics near 2x..6x with RPM-drift tolerance.
  harmonicsFound = 0;
  harmonicsPossible = 0;
  harmonicCoherence = 0.0f;
  if (fundFreq > 0.0f) {
    for (int h = 2; h <= MAX_HARMONIC; h++) {
      const float target = fundFreq * (float)h;
      if (target > BAND_MAX_HZ) break;
      harmonicsPossible++;
      int tol = (int)ceilf((target * HARMONIC_TOL_FRAC) / FREQ_RES);
      if (tol < 2) tol = 2;
      const int c = binOf(target);
      const float hMag = localPeakMagnitude(c, tol, bandLo, bandHi);
      if (hMag > HARMONIC_MIN_RATIO * fundMag &&
          hMag > HARMONIC_MIN_FLOOR * noiseFloorMag) {
        harmonicsFound++;
      }
    }
    if (harmonicsPossible > 0) {
      harmonicCoherence = (float)harmonicsFound / (float)harmonicsPossible;
    }
  }

  // --- Spectral spread: share of band POWER OUTSIDE the
  //     fundamental's own three bins. A clean sine keeps nearly
  //     everything inside; a drone spreads energy into harmonics
  //     and broadband prop wash.
  float bandPower = 0.0f;
  for (int b = bandLo; b <= bandHi; b++) {
    bandPower += vReal[b] * vReal[b];
  }

  outsideRatio = 0.0f;
  if (bandPower > 1e-6f && fundFreq > 0.0f) {
    float peakRegionPower = 0.0f;
    for (int b = peakBin - 1; b <= peakBin + 1; b++) {
      if (b >= bandLo && b <= bandHi) {
        peakRegionPower += vReal[b] * vReal[b];
      }
    }
    outsideRatio = 1.0f - (peakRegionPower / bandPower);
    if (outsideRatio < 0.0f) outsideRatio = 0.0f;
  }

}

// ============================================================
// MODULATION TRACKING (EMA mean/variance of f0 and energy)
// ============================================================

void resetModulation() {
  fMean = fundFreq; fVar = 0.0f;
  aMean = combinedEnergy; aVar = 0.0f;
  modSamples = 0;
}

void updateModulation() {
  if (fundFreq <= 0.0f) return;       // only track while a tone exists
  const float alpha = 0.05f;          // ~0.6 s time constant at 31 hops/s
  if (modSamples == 0) resetModulation();

  float d = fundFreq - fMean;
  fMean += alpha * d;
  fVar = (1.0f - alpha) * (fVar + alpha * d * d);

  d = combinedEnergy - aMean;
  aMean += alpha * d;
  aVar = (1.0f - alpha) * (aVar + alpha * d * d);

  if (modSamples < 0xFFFFFFF0UL) modSamples++;
}

// ============================================================
// FEATURE SCORES -> CONFIDENCE (normal drone mode)
// Weights: SNR 15, harmonics 25, spread 15, modulation 15,
// persistence 20, mic agreement 10. Energy alone caps at 15.
// ============================================================

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void resetEvidenceHistory() {
  memset(evidenceHistory, 0, sizeof(evidenceHistory));
  evidenceHistoryIndex = 0;
  evidenceHistoryCount = 0;
  evidenceHistoryGood = 0;
  evidenceDuty = 0.0f;
  dutyEvidenceOk = false;
}

void pushEvidenceHistory(bool strong) {
  const uint8_t value = strong ? 1 : 0;

  if (evidenceHistoryCount < EVIDENCE_HISTORY_WIN) {
    evidenceHistory[evidenceHistoryIndex] = value;
    evidenceHistoryCount++;
    evidenceHistoryGood += value;
  } else {
    evidenceHistoryGood -= evidenceHistory[evidenceHistoryIndex];
    evidenceHistory[evidenceHistoryIndex] = value;
    evidenceHistoryGood += value;
  }

  evidenceHistoryIndex++;
  if (evidenceHistoryIndex >= EVIDENCE_HISTORY_WIN) {
    evidenceHistoryIndex = 0;
  }

  evidenceDuty = (evidenceHistoryCount > 0)
      ? (float)evidenceHistoryGood / (float)evidenceHistoryCount : 0.0f;
  dutyEvidenceOk =
      (evidenceHistoryCount >= EVIDENCE_MIN_FILLED) &&
      (evidenceDuty >= EVIDENCE_DUTY_MIN);
}

void computeScores(uint32_t now) {
  // --- SNR (max 15): E relative to adaptive M
  const float snrRatio = combinedEnergy / thresholdM;   // M >= 1500, safe
  scores.snr = 15.0f * clampf((snrRatio - 1.0f) / 2.0f, 0.0f, 1.0f);

  // --- Harmonics (max 25): one isolated peak scores almost nothing
  switch (harmonicsFound) {
    case 0:  scores.harmonics = 0.0f;  break;
    case 1:  scores.harmonics = 6.0f;  break;
    case 2:  scores.harmonics = 14.0f; break;
    case 3:  scores.harmonics = 20.0f; break;
    default: scores.harmonics = 25.0f; break;
  }
  if (fundFreq <= 0.0f) scores.harmonics = 0.0f;

  // --- Spectral distribution (max 15): punish pure sine and
  //     shapeless noise, reward structured spread
  if (fundFreq <= 0.0f) {
    scores.distribution = 0.0f;
  } else if (outsideRatio >= NOISE_OUTSIDE_MAX) {
    scores.distribution = 4.0f;      // peak is meaningless in near-flat noise
  } else {
    scores.distribution = 15.0f * clampf(
      (outsideRatio - TONE_OUTSIDE_MIN) /
      (TONE_OUTSIDE_GOOD - TONE_OUTSIDE_MIN), 0.0f, 1.0f);
  }

  // --- Modulation (max 15): drones drift and correct RPM.
  //     A frozen frequency (sine) or wild hopping (music) score low.
  if (fundFreq <= 0.0f || modSamples < 16) {
    scores.modulation = 0.0f;
  } else {
    const float fStd = sqrtf(fVar);
    float freqScore;
    if (fStd <= MOD_FSTD_MIN_HZ) {
      freqScore = 0.0f;
    } else if (fStd < MOD_FSTD_FULL_HZ) {
      freqScore = 10.0f * (fStd - MOD_FSTD_MIN_HZ) /
                          (MOD_FSTD_FULL_HZ - MOD_FSTD_MIN_HZ);
    } else if (fStd <= MOD_FSTD_TAPER_HZ) {
      freqScore = 10.0f;
    } else if (fStd < MOD_FSTD_ZERO_HZ) {
      freqScore = 10.0f * (MOD_FSTD_ZERO_HZ - fStd) /
                          (MOD_FSTD_ZERO_HZ - MOD_FSTD_TAPER_HZ);
    } else {
      freqScore = 0.0f;
    }
    const float relAmp = sqrtf(aVar) / ((aMean > 1.0f) ? aMean : 1.0f);
    const float ampScore =
      (relAmp >= 0.03f && relAmp <= 0.35f) ? 5.0f
      : (relAmp < 0.03f ? 5.0f * (relAmp / 0.03f)
                        : 5.0f * clampf((0.7f - relAmp) / 0.35f, 0.0f, 1.0f));
    scores.modulation = clampf(freqScore + ampScore, 0.0f, 15.0f);
  }

  // --- Mic agreement (max 10): both mics must hear it, but a
  //     side source is legitimately louder on one mic.
  const float louder = (leftEnergy > rightEnergy) ? leftEnergy : rightEnergy;
  const float quieter = (leftEnergy > rightEnergy) ? rightEnergy : leftEnergy;
  if (louder < 1.0f) {
    scores.micAgree = 0.0f;
  } else {
    const float ratio = quieter / louder;
    scores.micAgree = 10.0f * clampf(
      (ratio - MIC_AGREE_MIN) / (MIC_AGREE_GOOD - MIC_AGREE_MIN), 0.0f, 1.0f);
  }

  // --- Per-window evidence + persistence (max 20)
  const bool aboveM = (combinedEnergy >= thresholdM);

  // Basic evidence opens a candidate and gives modulation tracking time to
  // settle. Strong evidence is deliberately stricter and is the only thing
  // allowed to increase persistence. This rejects music/video bursts that
  // occasionally resemble a drone but do not pass all features together.
  const bool basicEvidence =
      aboveM &&
      (fundFreq > 0.0f) &&
      (harmonicsFound >= REQUIRED_HARMONICS) &&
      (scores.micAgree >= MIC_EVIDENCE_MIN);

  if (basicEvidence && !candidateActive) {
    candidateActive = true;
    candidateStartMs = now;
    unstableWindows = 0;
    badWindows = 0;
    evidenceWindows = 0;
    resetEvidenceHistory();
    resetModulation();
  }

  strongWindowEvidence =
      basicEvidence &&
      (modSamples >= 16) &&
      (scores.distribution >= DIST_EVIDENCE_MIN) &&
      (scores.modulation >= MOD_EVIDENCE_MIN);

  if (candidateActive) {
    pushEvidenceHistory(strongWindowEvidence);
  } else {
    resetEvidenceHistory();
  }

  if (strongWindowEvidence) {
    evidenceWindows += PERSIST_GOOD_GAIN;
    const int cap = PERSIST_TARGET_WIN + PERSIST_TARGET_WIN / 5;
    if (evidenceWindows > cap) evidenceWindows = cap;
  } else {
    evidenceWindows -= PERSIST_BAD_DECAY;
    if (evidenceWindows < 0) evidenceWindows = 0;
    if (candidateActive) {
      if (aboveM) { if (unstableWindows < 999) unstableWindows++; }
      else        { if (badWindows < 999) badWindows++; }
    }
    if (evidenceWindows == 0 && state == ST_NO_DRONE && !basicEvidence) {
      candidateActive = false;
      resetEvidenceHistory();
    }
  }

  scores.persistence = 20.0f *
      clampf((float)evidenceWindows / (float)PERSIST_TARGET_WIN, 0.0f, 1.0f);

  // --- Total, clamped, smoothed
  // No sound below M may retain confidence merely because background noise
  // happens to contain harmonic peaks. This was the cause of CLEAR C:50 with
  // E near 1000 and M above 18000, which also blocked threshold recovery.
  float total = 0.0f;
  if (aboveM) {
    total = scores.snr + scores.harmonics + scores.distribution +
            scores.modulation + scores.persistence + scores.micAgree;
  } else {
    scores.snr = 0.0f;
    scores.harmonics = 0.0f;
    scores.distribution = 0.0f;
    scores.modulation = 0.0f;
    scores.micAgree = 0.0f;
  }
  scores.total = clampf(total, 0.0f, 100.0f);
  confSmoothed += CONF_SMOOTH_ALPHA * (scores.total - confSmoothed);
  confSmoothed = clampf(confSmoothed, 0.0f, 100.0f);

  if ((strongWindowEvidence || dutyEvidenceOk) &&
      confSmoothed >= EVIDENCE_CONF) {
    lastEvidenceMs = now;
  }
}

// ============================================================
// BACKGROUND ADAPTATION (frozen outside NO_DRONE, spec §4)
// ============================================================

void adaptBackground() {
  // Only learn background in NO_DRONE with no active candidate. Downward
  // recovery is deliberately faster so a noisy startup cannot leave M stuck.
  if (state != ST_NO_DRONE) return;
  if (candidateActive) return;
  if (combinedEnergy >= thresholdM) return;
  if (combinedEnergy > BG_ADAPT_MAX_RATIO * thresholdM) return;

  const float alpha = (combinedEnergy < bgEnergy)
      ? BG_ADAPT_DOWN_ALPHA : BG_ADAPT_UP_ALPHA;
  bgEnergy += alpha * (combinedEnergy - bgEnergy);
  float m = bgEnergy * (1.0f + CAL_MARGIN_MULT) + CAL_MARGIN_OFFSET;
  if (m < M_ABSOLUTE_FLOOR) m = M_ABSOLUTE_FLOOR;
  thresholdM = m;
}

// ============================================================
// STATE MACHINES (all timing via millis, non-blocking)
// ============================================================

void enterState(DetectorState s, uint32_t now) {
  state = s;
  stateEnteredMs = now;
  belowGradeMs = 0;
  droneGradeSinceMs = 0;
  if (s == ST_NO_DRONE) {
    evidenceWindows = 0;
    unstableWindows = 0;
    badWindows = 0;
    candidateActive = false;
    resetEvidenceHistory();
    confSmoothed *= 0.5f;   // decay, not hard reset: smoother display
  }
}

void updateNormalStateMachine(uint32_t now) {
  const uint32_t sinceEvidence = now - lastEvidenceMs;
  const uint32_t candidateMs = candidateActive ? (now - candidateStartMs) : 0;

  switch (state) {
    case ST_NO_DRONE:
      if (confSmoothed >= CONF_CHECKING && candidateActive &&
          candidateMs >= CHECKING_MIN_MS) {
        enterState(ST_CHECKING, now);
      }
      break;

    case ST_CHECKING:
      if (sinceEvidence > EVIDENCE_ABSENT_MS) {
        enterState(ST_NO_DRONE, now);
      } else if (dutyEvidenceOk &&
                 confSmoothed >= CONF_PROBABLE &&
                 candidateMs >= PROBABLE_MIN_MS) {
        enterState(ST_PROBABLE, now);
      }
      break;

    case ST_PROBABLE:
      if (sinceEvidence > EVIDENCE_ABSENT_MS) {
        enterState(ST_NO_DRONE, now);
      } else if (confSmoothed < CONF_PROBABLE) {
        droneGradeSinceMs = 0;
        if (belowGradeMs == 0) belowGradeMs = now;
        else if (now - belowGradeMs > DOWNGRADE_MS) enterState(ST_CHECKING, now);
      } else {
        belowGradeMs = 0;
        if (dutyEvidenceOk && confSmoothed >= CONF_DRONE &&
            candidateMs >= DRONE_MIN_MS) {
          if (droneGradeSinceMs == 0) droneGradeSinceMs = now;
          else if (now - droneGradeSinceMs >= DRONE_GRADE_HOLD_MS) {
            enterState(ST_DRONE, now);
          }
        } else {
          droneGradeSinceMs = 0;
        }
      }
      break;

    case ST_DRONE:
      if (sinceEvidence > EVIDENCE_ABSENT_MS) {
        enterState(ST_NO_DRONE, now);
      } else if (!dutyEvidenceOk || confSmoothed < CONF_PROBABLE) {
        if (belowGradeMs == 0) belowGradeMs = now;
        else if (now - belowGradeMs > DRONE_RELEASE_MS) {
          enterState(ST_PROBABLE, now);
        }
      } else {
        belowGradeMs = 0;
      }
      break;

    default:
      enterState(ST_NO_DRONE, now);
      break;
  }
}

// ============================================================
// DIRECTION (rough side bias only, spec §9)
// ============================================================

void updateDirection() {
  const float denom = leftEnergy + rightEnergy;
  if (denom < 1.0f) return;                          // no signal, hold bias
  if (combinedEnergy < DIR_MIN_ENERGY_FRAC * thresholdM) return;
  const float instant = (leftEnergy - rightEnergy) / denom;
  dirBias += DIR_SMOOTH_ALPHA * (instant - dirBias);
}

const char* directionText() {
  if (dirBias > DIR_BIAS_LIMIT)  return "LEFT";
  if (dirBias < -DIR_BIAS_LIMIT) return "RIGHT";
  return "CENTER";
}

// ============================================================
// MOTOR (GPIO 27, PN2222, wiring unchanged, no PWM)
// ============================================================

void updateMotor(uint32_t now) {
  const bool alarm = (state == ST_DRONE);

  if (!alarm) {
    if (motorOn) { digitalWrite(VIBRATION_PIN, LOW); motorOn = false; }
    motorPhaseMs = now;
    return;
  }

  // Pulsed buzz while the alarm state holds (kinder to motor + battery).
  if (motorOn) {
    if (now - motorPhaseMs >= MOTOR_PULSE_ON_MS) {
      digitalWrite(VIBRATION_PIN, LOW);
      motorOn = false;
      motorPhaseMs = now;
    }
  } else {
    if (now - motorPhaseMs >= MOTOR_PULSE_OFF_MS) {
      digitalWrite(VIBRATION_PIN, HIGH);
      motorOn = true;
      motorPhaseMs = now;
    }
  }
}

// ============================================================
// OLED (rate-limited, fits 128x64, spec §10)
// ============================================================

const char* stateText() {
  switch (state) {
    case ST_CALIBRATING: return "CALIBRATE";
    case ST_NO_DRONE:    return "NO DRONE";
    case ST_CHECKING:    return "CHECKING";
    case ST_PROBABLE:    return "PROBABLE";
    case ST_DRONE:       return "DRONE";
  }
  return "?";
}

void updateOLED(uint32_t now) {
  if (now - lastOledMs < OLED_UPDATE_MS) return;
  lastOledMs = now;

  char line[24];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (state == ST_CALIBRATING) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("CALIBRATE");
    display.setTextSize(1);
    uint32_t elapsed = now - calStartMs;
    uint32_t remain = (elapsed < CALIBRATION_MS) ? (CALIBRATION_MS - elapsed) : 0;
    snprintf(line, sizeof(line), "Time left: %lu.%lus",
             (unsigned long)(remain / 1000),
             (unsigned long)((remain % 1000) / 100));
    display.setCursor(0, 24);
    display.print(line);
    float bgNow = (calCount > 0) ? (float)(calSum / calCount) : 0.0f;
    snprintf(line, sizeof(line), "Bg: %d", (int)bgNow);
    display.setCursor(0, 36);
    display.print(line);
    display.setCursor(0, 52);
    display.print(POWER_SOURCE_POWERBANK ? "Pwr: USB" : "Pwr: BAT:--");
    display.display();
    return;
  }

  // Row 1: short status + confidence.
  // Long labels at text size 2 would overlap the right-hand value.
  display.setTextSize(2);
  display.setCursor(0, 0);
  const char* shortStatus;
  switch (state) {
    case ST_NO_DRONE:  shortStatus = "CLEAR"; break;
    case ST_CHECKING:  shortStatus = "POSS";  break;
    case ST_PROBABLE:  shortStatus = "POSS";  break;
    case ST_DRONE:     shortStatus = "DRONE"; break;
    default:           shortStatus = "WAIT";  break;
  }
  display.print(shortStatus);
  display.setTextSize(1);
  snprintf(line, sizeof(line), "C:%d", (int)confSmoothed);
  display.setCursor(SCREEN_WIDTH - 6 * strlen(line), 4);
  display.print(line);

  // Row 2: E and M
  snprintf(line, sizeof(line), "E:%-6d M:%d",
           (int)combinedEnergy, (int)thresholdM);
  display.setCursor(0, 20);
  display.print(line);

  // Row 3: direction + tonal/harmonic flag
  snprintf(line, sizeof(line), "Dir:%-6s T:%c",
           directionText(), dutyEvidenceOk ? 'Y' : 'N');
  display.setCursor(0, 32);
  display.print(line);

  // Row 4: window counters + candidate duration
  const float candSec = candidateActive
      ? (float)(now - candidateStartMs) / 1000.0f : 0.0f;
  snprintf(line, sizeof(line), "U:%-3u B:%-3u %d.%ds",
           unstableWindows, badWindows,
           (int)candSec, (int)(candSec * 10.0f) % 10);
  display.setCursor(0, 44);
  display.print(line);

  // Row 5: live dominant background frequency + power source.
  // This reader is informational and does not by itself identify a drone.
  display.setCursor(0, 56);
  snprintf(line, sizeof(line), "F:%dHz", (int)lroundf(livePeakFreq));
  display.print(line);
  display.setCursor(98, 56);
  display.print(POWER_SOURCE_POWERBANK ? "USB" : "BAT");
  display.display();
}

// ============================================================
// SERIAL DIAGNOSTICS (rate-limited, spec §13)
// ============================================================

void debugPrint(uint32_t now) {
#if DEBUG_MODE
#if SERIAL_PLOTTER_MODE
  if (now - lastDebugMs < PLOTTER_PRINT_MS) return;
  lastDebugMs = now;

  // Normalize all traces to a comparable 0..100 scale. Energy may rise to
  // 300 so loud events remain visible without flattening the feature plots.
  const float energyPct = clampf(
      100.0f * combinedEnergy / ((thresholdM > 1.0f) ? thresholdM : 1.0f),
      0.0f, 300.0f);
  float statePct = 0.0f;
  switch (state) {
    case ST_NO_DRONE: statePct = 0.0f; break;
    case ST_CHECKING: statePct = 33.0f; break;
    case ST_PROBABLE: statePct = 66.0f; break;
    case ST_DRONE: statePct = 100.0f; break;
    default: statePct = 0.0f; break;
  }

  Serial.print("Energy:");     Serial.print(energyPct, 1);
  Serial.print("\tThreshold:"); Serial.print(100.0f, 1);
  Serial.print("\tConfidence:");Serial.print(confSmoothed, 1);
  Serial.print("\tHarmonics:"); Serial.print(scores.harmonics * 4.0f, 1);
  Serial.print("\tHarmonicFit:");Serial.print(harmonicCoherence * 100.0f, 1);
  Serial.print("\tSpectrum:");  Serial.print(scores.distribution * (100.0f / 15.0f), 1);
  Serial.print("\tModulation:");Serial.print(scores.modulation * (100.0f / 15.0f), 1);
  Serial.print("\tPersistence:");Serial.print(scores.persistence * 5.0f, 1);
  Serial.print("\tMicAgree:");  Serial.print(scores.micAgree * 10.0f, 1);
  Serial.print("\tHardGate:");  Serial.print(strongWindowEvidence ? 100 : 0);
  Serial.print("\tGateDuty:");  Serial.print(evidenceDuty * 100.0f, 1);
  Serial.print("\tState:");     Serial.println(statePct, 1);
#else
  if (now - lastDebugMs < DEBUG_PRINT_MS) return;
  lastDebugMs = now;

  Serial.print("st=");    Serial.print(stateText());
  Serial.print(" L=");    Serial.print((int)leftEnergy);
  Serial.print(" R=");    Serial.print((int)rightEnergy);
  Serial.print(" E=");    Serial.print((int)combinedEnergy);
  Serial.print(" M=");    Serial.print((int)thresholdM);
  Serial.print(" liveF=");Serial.print(livePeakFreq, 1);
  Serial.print(" f0=");   Serial.print(fundFreq, 1);
  Serial.print(" harm="); Serial.print(scores.harmonics, 0);
  Serial.print(" fit=");  Serial.print(harmonicCoherence * 100.0f, 0);
  Serial.print(" dist="); Serial.print(scores.distribution, 0);
  Serial.print(" mod=");  Serial.print(scores.modulation, 0);
  Serial.print(" pers="); Serial.print(scores.persistence, 0);
  Serial.print(" mic=");  Serial.print(scores.micAgree, 0);
  Serial.print(" gate="); Serial.print(strongWindowEvidence ? 'Y' : 'N');
  Serial.print(" duty="); Serial.print(evidenceDuty * 100.0f, 0);
  Serial.print(" C=");    Serial.print(confSmoothed, 0);
  Serial.print(" bias="); Serial.print(dirBias, 2);
  Serial.println();
#endif
#else
  (void)now;
#endif
}

// ============================================================
// CALIBRATION (5 s, motor off, spec §4)
// ============================================================

void runCalibrationStep(uint32_t now) {
  // Ignore the first half-second while I2S DMA and microphone readings settle.
  if (now - calStartMs >= CAL_SETTLE_MS) {
    calSum += combinedEnergy;
    calSumSq += (double)combinedEnergy * (double)combinedEnergy;
    calCount++;
  }

  if (now - calStartMs >= CALIBRATION_MS && calCount >= 10) {
    const double mean = calSum / calCount;
    double var = (calSumSq / calCount) - mean * mean;
    if (var < 0.0) var = 0.0;
    const double sd = sqrt(var);

    bgEnergy = (float)mean;
    double margin = CAL_STD_MULT * sd;
    const double relMargin = mean * CAL_MARGIN_MULT;
    const double maxMargin = mean * CAL_MAX_MARGIN_MULT;
    if (relMargin > margin) margin = relMargin;
    if (margin > maxMargin) margin = maxMargin;
    double m = mean + margin + CAL_MARGIN_OFFSET;
    if (m < M_ABSOLUTE_FLOOR) m = M_ABSOLUTE_FLOOR;
    thresholdM = (float)m;

    enterState(ST_NO_DRONE, now);
#if DEBUG_MODE && !SERIAL_PLOTTER_MODE
    Serial.print("Calibration done. bg=");
    Serial.print(bgEnergy, 0);
    Serial.print(" sd=");
    Serial.print(sd, 0);
    Serial.print(" M=");
    Serial.println(thresholdM, 0);
#endif
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);

  memset(leftFftBuf, 0, sizeof(leftFftBuf));
  memset(rightFftBuf, 0, sizeof(rightFftBuf));

  setupOLED();
  if (!setupI2S()) {
    haltWithError("I2S ERROR", "Check Serial");
  }

  calStartMs = millis();
  stateEnteredMs = calStartMs;
  state = ST_CALIBRATING;

#if DEBUG_MODE && !SERIAL_PLOTTER_MODE
  Serial.println();
  Serial.println("Drone Detector MVP | normal detection mode");
#endif
}

void loop() {
  // Pacing comes from i2s_read: it returns when 32 ms of new audio
  // is available. No delay() anywhere in the runtime path.
  readAudioHop();
  const uint32_t now = millis();

  if (state == ST_CALIBRATING) {
    runCalibrationStep(now);
    updateMotor(now);          // guarantees motor stays off
    updateOLED(now);
    return;
  }

  runFFT();
  analyzeSpectrum();
  updateModulation();
  updateDirection();

  computeScores(now);
  updateNormalStateMachine(now);

  adaptBackground();
  updateMotor(now);
  updateOLED(now);
  debugPrint(now);
}