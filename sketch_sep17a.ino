/*
  GP2Y0A02YK0F -> analog low-pass -> ADS1115 -> Arduino Uno (I2C)
  Goal: maintain distance x_set_cm using PID + gentle feed-forward.
  Output: 5V PWM on pin 9 at ~8.2 kHz (drive a MOSFET/H-bridge, not the motor directly!)

  Library: "Adafruit ADS1X15" by Adafruit (plus "Adafruit BusIO")
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

Adafruit_ADS1115 ads;

// ===================== Adjust here =====================
/*** Setpoint & behavior ***/
float x_set_cm        = 65.0;     // distance target (cm)
const float STARTUP_DUTY = 0.50;  // open-loop duty for startup window (0..1)
const unsigned long STARTUP_MS = 3000UL;

/*** PID gains ***/
float Kp = 0.01;
float Ki = 0.00;
float Kd = 0.2;

/*** Control direction ***/
// Set to -1 so that duty INCREASES when voltage DECREASES (your requirement).
// If your mechanics act the other way, change to +1.
const int PID_SIGN = -1;

/*** Feed-forward on voltage error (helps anticipate changes) ***/
float Kff = 0.10;                 // set 0 to disable
// Feed-forward uses (y_target - y_filtered); positive when actual voltage is low.
// That naturally increases duty to match your requirement, so no sign flip here.

/*** Duty cycle limits ***/
float DUTY_MIN = 0.45;            // 10%
float DUTY_MAX = 0.55;            // 90%

/*** Filtering knobs ***/
float emaAlpha = 0.20f;           // 0..1, higher = snappier, lower = smoother
float spikeVoltThreshold = 0.35f; // ignore sudden jumps > this (V)
bool  useSpikeClamp = true;

/*** ADS1115 channel and data rate ***/
int adsChannel = 0;               // AIN0
const uint16_t ADS_RATE = RATE_ADS1115_475SPS;  // or RATE_ADS1115_860SPS
// ======================================================

// Equation constants (y volts <-> x cm)   y(x) = A/(x+B) + C
const float A_c = 83.3907f;
const float B_c = 11.6889f;
const float C_c = -0.07427f;

// Timer1 for ~8.2 kHz PWM on pin 9 (OC1A)
const uint16_t PWM_TOP = 1951;  // 16e6 / (1*(1+TOP)) ≈ 8.2 kHz

#define PWM_PIN 9
#define DEBUG 0  // set to 1 for Serial debug

// --- state ---
float yMedian = 0, yEMA = 0, yFiltered = 0;
float lastError = 0, integ = 0;
unsigned long lastUpdateUs = 0;
const unsigned long CONTROL_PERIOD_US = 5000; // 200 Hz control (5 ms)

float median5(float v);
float y_from_x(float x_cm);
float x_from_y(float y_v);
void  pwmSetup();
void  setDuty(float duty01);
float readSensorVolt();

void setup() {
  #if DEBUG
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("Starting..."));
  #endif

  // I2C / ADS1115
  Wire.begin(); // use the Uno's SDA/SCL-labeled header
  if (!ads.begin()) {
    #if DEBUG
      Serial.println(F("ADS1115 not found! Check wiring/power."));
    #endif
    while (1) { delay(100); }
  }
  ads.setGain(GAIN_ONE);            // +/- 4.096 V (good for ~0..3V sensor)
  ads.setDataRate(ADS_RATE);        // speed vs noise tradeoff

  // Prime filters with first reading
  float y0 = readSensorVolt();
  yMedian = y0;
  yEMA    = y0;
  yFiltered = y0;

  // PWM @ ~8.2 kHz on pin 9
  pwmSetup();

  lastUpdateUs = micros();
}

void loop() {
  unsigned long now = micros();

  // --- Open-loop startup window ---
  if (millis() < STARTUP_MS) {
    setDuty(STARTUP_DUTY);
    return;
  }

  // --- Run control at fixed rate ---
  if (now - lastUpdateUs < CONTROL_PERIOD_US) return;
  float dt = (now - lastUpdateUs) * 1e-6f; // seconds
  lastUpdateUs = now;

  // --- Measure & filter voltage ---
  float y = readSensorVolt();        // raw volts

  // median-of-5 to crush sharp spikes
  float yMed = median5(y);

  // EMA low-pass
  yEMA = (1.0f - emaAlpha) * yEMA + emaAlpha * yMed;

  // Optional spike clamp: ignore absurd jumps even after median
  if (useSpikeClamp && fabsf(yMed - yEMA) > spikeVoltThreshold) {
    yFiltered = yEMA; // hold near EMA
  } else {
    yFiltered = yEMA;
  }

  // --- Convert to estimated distance (cm) ---
  float x_meas_cm = x_from_y(yFiltered);

  // Keep within sensor’s reliable band (optional)
  if (x_meas_cm < 20)  x_meas_cm = 20;
  if (x_meas_cm > 150) x_meas_cm = 150;

  // --- PID on distance error (cm) ---
  float error = x_set_cm - x_meas_cm;

  // Integral with anti-windup (only integrate if not saturating badly)
  float dutyNow = (float)OCR1A / (float)PWM_TOP;
  bool atUpper = dutyNow >= (DUTY_MAX - 0.001f);
  bool atLower = dutyNow <= (DUTY_MIN + 0.001f);
  if (!(atUpper && error > 0) && !(atLower && error < 0)) {
    integ += error * dt;
  }

  float deriv = (error - lastError) / dt;
  lastError = error;

  // Apply PID_SIGN so duty increases when voltage decreases
  float u_pid = PID_SIGN * (Kp * error + Ki * integ + Kd * deriv);

  // --- Feed-forward using voltage error (anticipate changes) ---
  float y_target = y_from_x(x_set_cm);
  float eV = y_target - yFiltered;   // positive when measured voltage is low
  float u_ff = Kff * eV;             // naturally increases duty when voltage drops

  // Combine & map to duty. Start from current duty to avoid big steps.
  float dutyCmd = dutyNow + u_pid + u_ff;

  // Clamp to configured limits
  if (dutyCmd < DUTY_MIN) dutyCmd = DUTY_MIN;
  if (dutyCmd > DUTY_MAX) dutyCmd = DUTY_MAX;

  setDuty(dutyCmd);

  #if DEBUG
    static uint32_t tms = 0;
    if (millis() - tms > 200) {
      tms = millis();
      Serial.print(F("y(V)="));   Serial.print(yFiltered, 3);
      Serial.print(F("  x(cm)="));Serial.print(x_meas_cm, 1);
      Serial.print(F("  err="));  Serial.print(error, 2);
      Serial.print(F("  duty=")); Serial.println(dutyCmd, 3);
    }
  #endif
}

// ======= Helpers =======

// Sensor model: y(x) = A/(x+B) + C
float y_from_x(float x_cm) {
  return (A_c / (x_cm + B_c)) + C_c;
}

// Inverse: x(y) = A/(y - C) - B
float x_from_y(float y_v) {
  float denom = (y_v - C_c);
  // Prevent division by ~0 near asymptote
  if (fabsf(denom) < 1e-4f) denom = (denom >= 0 ? 1e-4f : -1e-4f);
  return (A_c / denom) - B_c;
}

// Median of 5 (small ring buffer)
float median5(float v) {
  static float buf[5];
  static uint8_t idx = 0;
  static bool primed = false;

  if (!primed) {
    for (int i = 0; i < 5; ++i) buf[i] = v;
    primed = true;
  }
  buf[idx] = v;
  idx = (idx + 1) % 5;

  float a[5];
  for (int i = 0; i < 5; ++i) a[i] = buf[i];

  // insertion sort small array
  for (int i = 1; i < 5; ++i) {
    float key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) { a[j+1] = a[j]; j--; }
    a[j+1] = key;
  }
  return a[2];
}

float readSensorVolt() {
  int16_t raw = ads.readADC_SingleEnded(adsChannel);
  // Convert counts to volts using library helper (respects current gain)
  float y = ads.computeVolts(raw);
  if (y < 0) y = 0;
  return y;
}

void pwmSetup() {
  pinMode(PWM_PIN, OUTPUT);
  // Timer1 Fast PWM, mode 14 (WGM13:0 = 1110), non-inverting on OC1A
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1);            // OC1A non-inverting
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  ICR1 = PWM_TOP;                     // TOP -> sets frequency (~8.2 kHz)
  TCCR1B |= (1 << CS10);              // prescaler 1
  OCR1A = (uint16_t)(STARTUP_DUTY * PWM_TOP); // initial
}

void setDuty(float duty01) {
  if (duty01 < 0) duty01 = 0;
  if (duty01 > 1) duty01 = 1;
  OCR1A = (uint16_t)(duty01 * PWM_TOP + 0.5f);
}
