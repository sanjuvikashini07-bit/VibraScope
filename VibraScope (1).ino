/*
  ============================================================================
  VibraScope - Portable Machine Vibration Diagnostic System
  Hardware: ESP32 DevKit V1, MPU6050, 0.96" SSD1306 OLED, Buzzer/Speaker,
            3x push buttons (Start/Stop, Calibrate, Screen Mode)

  See VibraScope_Documentation.md for wiring, principle, calibration and
  testing procedure. This file is organized into clearly labeled SECTIONS
  that match the module descriptions in that document.
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <arduinoFFT.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ============================================================================
// SECTION: Configuration & Globals
// ============================================================================

// ---- I2C / OLED ----
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_RESET   -1
#define OLED_ADDR    0x3C
#define I2C_SDA      21
#define I2C_SCL      22
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// ---- MPU6050 ----
Adafruit_MPU6050 mpu;

// ---- Buttons ----
#define BTN_STARTSTOP 32
#define BTN_CALIB     33
#define BTN_SCREEN    25
#define DEBOUNCE_MS   200

// ---- Buzzer ----
#define BUZZER_PIN 26

// ---- Sampling / FFT configuration ----
#define SAMPLES          256              // must be a power of 2 for arduinoFFT
#define SAMPLING_FREQ_HZ  500              // fixed sample rate
#define SAMPLE_INTERVAL_US (1000000UL / SAMPLING_FREQ_HZ)

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLES, (double)SAMPLING_FREQ_HZ);

// ---- Status thresholds (tune during bench testing) ----
#define WARNING_RMS_RATIO   1.5   // current RMS > 1.5x baseline -> WARNING
#define HIGH_RMS_RATIO      2.5   // current RMS > 2.5x baseline -> HIGH
#define HIGH_PEAK_RATIO     3.0   // current Peak > 3.0x baseline -> HIGH
#define FREQ_DEV_WARN_PCT   15.0  // dominant freq shifted >15% from baseline -> WARNING

#define CALIB_CYCLES 5            // number of buffers averaged during calibration

// ---- System state machine ----
enum SystemState { IDLE_STATE, RUNNING_STATE, CALIBRATING_STATE };
SystemState state = IDLE_STATE;

int screenMode = 0;               // 0 = main, 1 = baseline compare, 2 = FFT bars
#define NUM_SCREENS 3

// ---- Baseline (healthy signature) ----
float baseRMS = 0, basePeak = 0, baseFreq = 0;
bool  baselineSet = false;

// ---- Current live readings ----
float currentRMS = 0, currentPeak = 0, currentFreq = 0;
String statusText = "IDLE";

// ---- Preferences (flash storage for baseline) ----
Preferences prefs;

// ---- Button debounce bookkeeping ----
unsigned long lastDebounce_StartStop = 0;
unsigned long lastDebounce_Calib     = 0;
unsigned long lastDebounce_Screen    = 0;

// ============================================================================
// SECTION: WiFi / Dashboard Streaming
// ============================================================================
const char* WIFI_SSID = "YourWiFiName";
const char* WIFI_PASS = "YourWiFiPassword";

WebServer server(80);

// Builds and sends the current reading as JSON when the dashboard requests
// GET http://<esp32-ip>/data
void handleData() {
  StaticJsonDocument<256> doc;
  doc["timestamp"] = millis();
  doc["rms"] = currentRMS;
  doc["peak"] = currentPeak;
  doc["frequency"] = currentFreq;
  if (baselineSet) {
    doc["baseline_rms"] = baseRMS;
    doc["baseline_peak"] = basePeak;
    doc["baseline_frequency"] = baseFreq;
  }
  doc["status"] = statusText;

  String json;
  serializeJson(doc, json);
  server.sendHeader("Access-Control-Allow-Origin", "*"); // allow browser fetch from dashboard
  server.send(200, "application/json", json);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);

  const int MAX_ATTEMPTS = 3;
  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    WiFi.disconnect(true, true);  // clear any stale/cached config from previous networks
    delay(1000);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to WiFi (attempt %d/%d)", attempt, MAX_ATTEMPTS);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(400);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) break;
    Serial.print("\nAttempt failed - status code: ");
    Serial.println(WiFi.status());    // 1=NO_SSID_AVAIL 4=CONNECT_FAILED 5=CONN_LOST 6=DISCONNECTED
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nESP32 IP: ");
    Serial.println(WiFi.localIP());   // <-- note this down, dashboard needs it
    server.on("/data", handleData);
    server.begin();
  } else {
    Serial.println("\nWiFi not connected after retries - continuing standalone (no dashboard).");
  }
}

// ============================================================================
// SECTION: Setup
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(BTN_STARTSTOP, INPUT_PULLUP);
  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_SCREEN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 not found - check wiring/address");
    while (true) delay(1000);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("VibraScope");
  display.println("Initializing...");
  display.display();

  // MPU6050 init
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found - check wiring");
    display.println("MPU6050 FAIL!");
    display.display();
    while (true) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); // built-in low-pass, reduces sensor noise

  // Load saved baseline (if any) from flash
  prefs.begin("vibra", false);
  if (prefs.isKey("baseRMS")) {
    baseRMS  = prefs.getFloat("baseRMS", 0);
    basePeak = prefs.getFloat("basePeak", 0);
    baseFreq = prefs.getFloat("baseFreq", 0);
    baselineSet = true;
    Serial.println("Loaded saved baseline from flash.");
  }

  // Connect WiFi and start the dashboard WebSocket server
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.display();
  setupWiFi();

  delay(800);
  state = IDLE_STATE;
  updateDisplay();
}

// ============================================================================
// SECTION: Button Handling
// Non-blocking, debounced. Runs every loop() pass regardless of system state.
// ============================================================================
void handleButtons() {
  unsigned long now = millis();

  // --- Start/Stop button ---
  if (digitalRead(BTN_STARTSTOP) == LOW && (now - lastDebounce_StartStop) > DEBOUNCE_MS) {
    lastDebounce_StartStop = now;
    if (state == IDLE_STATE) {
      state = RUNNING_STATE;
      statusText = "STARTING";
    } else if (state == RUNNING_STATE) {
      state = IDLE_STATE;
      statusText = "IDLE";
      digitalWrite(BUZZER_PIN, LOW);
      updateDisplay();
    }
    // ignored while CALIBRATING_STATE (let calibration finish)
  }

  // --- Calibrate button (only meaningful from IDLE) ---
  if (digitalRead(BTN_CALIB) == LOW && (now - lastDebounce_Calib) > DEBOUNCE_MS) {
    lastDebounce_Calib = now;
    if (state == IDLE_STATE) {
      state = CALIBRATING_STATE;
    }
  }

  // --- Screen mode button ---
  if (digitalRead(BTN_SCREEN) == LOW && (now - lastDebounce_Screen) > DEBOUNCE_MS) {
    lastDebounce_Screen = now;
    screenMode = (screenMode + 1) % NUM_SCREENS;
    if (state == IDLE_STATE) updateDisplay(); // refresh immediately if idle
  }
}

// ============================================================================
// SECTION: Acquisition
// Fills vReal[] with DC/gravity-removed vibration samples at a fixed rate.
// Blocking for ~ SAMPLES / SAMPLING_FREQ_HZ seconds (≈512 ms) by design -
// this keeps sample timing precise and simple for a student project.
// ============================================================================
void acquireBuffer() {
  static double raw[SAMPLES];
  sensors_event_t a, g, temp;

  unsigned long nextSampleTime = micros();

  for (int i = 0; i < SAMPLES; i++) {
    while ((long)(micros() - nextSampleTime) < 0) {
      // busy-wait until the next fixed sample instant
    }
    mpu.getEvent(&a, &g, &temp);
    // Using Z axis: assumes sensor mounted flat on top of motor/fan housing.
    // If your mounting orientation is different, use a.acceleration.x or .y,
    // or switch to magnitude = sqrt(x^2+y^2+z^2) for orientation independence.
    raw[i] = a.acceleration.z; // m/s^2
    nextSampleTime += SAMPLE_INTERVAL_US;
  }

  // ---- DC / gravity offset removal: subtract buffer mean ----
  double mean = 0;
  for (int i = 0; i < SAMPLES; i++) mean += raw[i];
  mean /= SAMPLES;

  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = raw[i] - mean;  // zero-mean AC vibration signal
    vImag[i] = 0;
  }
}

// ============================================================================
// SECTION: Time-Domain Metrics
// Must be called on the un-windowed AC signal, BEFORE runFFT() (which
// modifies vReal in place via windowing).
// ============================================================================
void computeRmsPeak(float &rmsOut, float &peakOut) {
  double sumSq = 0, peak = 0;
  for (int i = 0; i < SAMPLES; i++) {
    double s = vReal[i];
    sumSq += s * s;
    double a = fabs(s);
    if (a > peak) peak = a;
  }
  rmsOut  = sqrt(sumSq / SAMPLES);
  peakOut = peak;
}

// ============================================================================
// SECTION: Frequency-Domain Analysis
// Applies Hamming window, runs FFT, extracts dominant frequency.
// Leaves vReal[] holding the magnitude spectrum afterwards (used by the
// FFT bar-graph display screen).
// ============================================================================
float runFFT() {
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);
  double peakFreq = FFT.majorPeak(vReal, SAMPLES, (double)SAMPLING_FREQ_HZ);
  return (float)peakFreq;
}

// ============================================================================
// SECTION: Status Evaluation
// Compares current readings to the stored healthy baseline.
// ============================================================================
void evaluateStatus() {
  if (!baselineSet) {
    statusText = "NO BASELINE";
    return;
  }

  float rmsRatio  = (baseRMS  > 0.0001) ? currentRMS  / baseRMS  : 0;
  float peakRatio = (basePeak > 0.0001) ? currentPeak / basePeak : 0;
  float freqDevPct = (baseFreq > 0.0001)
                        ? fabs(currentFreq - baseFreq) / baseFreq * 100.0
                        : 0;

  if (rmsRatio > HIGH_RMS_RATIO || peakRatio > HIGH_PEAK_RATIO) {
    statusText = "HIGH VIBRATION";
  } else if (rmsRatio > WARNING_RMS_RATIO || freqDevPct > FREQ_DEV_WARN_PCT) {
    statusText = "WARNING";
  } else {
    statusText = "NORMAL";
  }
}

// ============================================================================
// SECTION: Calibration
// Captures CALIB_CYCLES buffers while the machine is assumed healthy,
// averages RMS/Peak/Freq, stores as the new baseline (RAM + flash).
// ============================================================================
void runCalibration() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.println("Keep motor running");
  display.display();

  float sumRMS = 0, sumPeak = 0, sumFreq = 0;

  for (int c = 0; c < CALIB_CYCLES; c++) {
    acquireBuffer();
    float rms, peak;
    computeRmsPeak(rms, peak);
    float freq = runFFT();

    sumRMS  += rms;
    sumPeak += peak;
    sumFreq += freq;

    display.fillRect(0, 20, OLED_WIDTH, 10, SSD1306_BLACK);
    display.setCursor(0, 20);
    display.print("Pass ");
    display.print(c + 1);
    display.print("/");
    display.println(CALIB_CYCLES);
    display.display();
  }

  baseRMS  = sumRMS  / CALIB_CYCLES;
  basePeak = sumPeak / CALIB_CYCLES;
  baseFreq = sumFreq / CALIB_CYCLES;
  baselineSet = true;

  // Persist to flash so it survives power-off
  prefs.putFloat("baseRMS", baseRMS);
  prefs.putFloat("basePeak", basePeak);
  prefs.putFloat("baseFreq", baseFreq);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration Done");
  display.print("RMS: "); display.println(baseRMS, 2);
  display.print("Peak: "); display.println(basePeak, 2);
  display.print("Freq: "); display.print(baseFreq, 1); display.println(" Hz");
  display.display();
  delay(1500);

  statusText = "IDLE";
  state = IDLE_STATE;
}

// ============================================================================
// SECTION: Buzzer Alert
// NORMAL -> silent | WARNING -> intermittent beep | HIGH -> continuous tone
// Called once per RUNNING cycle (~0.5-1s), which is enough to produce an
// audibly distinct slow-beep vs continuous-tone pattern.
// ============================================================================
void updateBuzzer() {
  static bool toggle = false;
  static unsigned long lastToggle = 0;

  if (statusText == "HIGH VIBRATION") {
    digitalWrite(BUZZER_PIN, HIGH); // continuous
  } else if (statusText == "WARNING") {
    if (millis() - lastToggle > 300) { // ~300ms on/off blink
      toggle = !toggle;
      lastToggle = millis();
    }
    digitalWrite(BUZZER_PIN, toggle ? HIGH : LOW);
  } else {
    digitalWrite(BUZZER_PIN, LOW); // NORMAL / IDLE = silent
  }
}

// ============================================================================
// SECTION: Display
// Draws one of NUM_SCREENS OLED screens depending on screenMode.
// ============================================================================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (state == IDLE_STATE) {
    display.println("VibraScope - IDLE");
    display.println("Start: btn1");
    display.println("Calib: btn2 (motor ON)");
    display.println("Screen: btn3");
    if (baselineSet) {
      display.println("");
      display.print("Baseline RMS: ");
      display.println(baseRMS, 2);
    } else {
      display.println("No baseline set!");
    }
    display.display();
    return;
  }

  switch (screenMode) {
    case 0: { // Main readings
      display.setTextSize(1);
      display.println("VibraScope - LIVE");
      display.print("RMS : "); display.print(currentRMS, 2); display.println(" m/s2");
      display.print("Peak: "); display.print(currentPeak, 2); display.println(" m/s2");
      display.print("Freq: "); display.print(currentFreq, 1); display.println(" Hz");
      display.setTextSize(2);
      display.setCursor(0, 44);
      display.println(statusText);
      break;
    }
    case 1: { // Baseline comparison
      display.println("Baseline Compare");
      display.print("Base RMS : "); display.println(baseRMS, 2);
      display.print("Now  RMS : "); display.println(currentRMS, 2);
      display.print("Base Frq : "); display.print(baseFreq, 1); display.println(" Hz");
      display.print("Now  Frq : "); display.print(currentFreq, 1); display.println(" Hz");
      display.print("Status: "); display.println(statusText);
      break;
    }
    case 2: { // Mini FFT bar graph (vReal currently holds magnitude spectrum)
      display.println("Spectrum (0-125Hz)");
      int numBars = 32;
      int barWidth = OLED_WIDTH / numBars;
      // find max among first 128 bins (Nyquist half) for scaling, skip bin 0 (DC)
      double maxMag = 0.0001;
      for (int i = 1; i < SAMPLES / 2; i++) {
        if (vReal[i] > maxMag) maxMag = vReal[i];
      }
      for (int b = 0; b < numBars; b++) {
        int bin = 1 + b * ((SAMPLES / 2 - 1) / numBars); // spread across spectrum
        int barHeight = (int)((vReal[bin] / maxMag) * 40.0);
        if (barHeight > 40) barHeight = 40;
        int x = b * barWidth;
        int y = 63 - barHeight;
        display.fillRect(x, y, barWidth - 1, barHeight, SSD1306_WHITE);
      }
      break;
    }
  }
  display.display();
}

// ============================================================================
// SECTION: Main Loop (state machine)
// ============================================================================
void loop() {
  server.handleClient();
  handleButtons();

  switch (state) {
    case IDLE_STATE:
      // idle screen only needs redrawing on button events (handled above);
      // small delay keeps CPU/button polling responsive without busy-spin
      delay(50);
      break;

    case CALIBRATING_STATE:
      runCalibration(); // blocking; returns with state set back to IDLE_STATE
      break;

    case RUNNING_STATE:
      acquireBuffer();
      computeRmsPeak(currentRMS, currentPeak);
      currentFreq = runFFT();
      evaluateStatus();
      updateBuzzer();
      updateDisplay();
      server.handleClient(); // serve /data again now, with THIS cycle's fresh values
      break;
  }
}
