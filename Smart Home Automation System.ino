/*
  ESP32 Smart Automation System
  Board  : ESP32 DevKit (Arduino IDE -> "ESP32 Dev Module")
  Core   : Arduino-ESP32 2.0.x or 3.x (LEDC API handled automatically)
  Libs   : ESP32Servo, PCF8574 (Rob Tillaart)

  ASSUMPTIONS (change below if wrong):
   1. Temperature sensor  = LM35 (10 mV per deg C), analog output on GPIO 32.
   2. Rain sensor         = YL-83 / FC-37 style module powered from 3.3 V.
                            Its analog value DECREASES when wet (RAIN_WET_WHEN_LOWER = true).
   3. MQ gas sensor       = MQ-x module, AO through a divider (see wiring notes).
   4. Buzzer              = ACTIVE buzzer (built-in oscillator), driven HIGH = ON.
   5. Ultrasonic sensors  = HC-SR04 (ECHO through a 5V -> 3.3V divider).
   6. Keypad              = 4x4 matrix on PCF8574 @ 0x20:
                            Rows R1..R4 -> P0..P3 , Columns C1..C4 -> P4..P7
                            Keys: 1 2 3 A / 4 5 6 B / 7 8 9 C / * 0 # D
                            '#' = confirm password, '*' = clear entry.
                            Scan logic merged from the working keypad test sketch.

  RAIN / MOTOR 2 BEHAVIOUR:
   Motor 2 stays OFF until the rain state crosses the threshold.
   Dry -> Wet : Motor 2 runs FORWARD for MOTOR2_RUN_MS (5 s), then stops.
   Wet -> Dry : Motor 2 runs REVERSE for MOTOR2_RUN_MS (5 s), then stops.

  BLYNK CLOUD (read-only dashboard):
   All sensor values are pushed to Blynk every BLYNK_SEND_MS. Local control never
   waits for WiFi/Blynk: if the connection drops, everything keeps working and
   the sketch reconnects in the background.
   Extra lib: "Blynk" by Volodymyr Shymanskyy.
   ADC note : only ADC1 pins (32, 35, 39) are used for analog, so WiFi does not clash.

   Virtual pins (create these Datastreams in the Blynk template):
    V0  Temperature (C)      double   V1  Rain raw         integer
    V2  Rain state           string   V3  Gas raw          integer
    V4  Gas alarm            string   V5  Distance 1 (cm)  double  (-1 = no echo)
    V6  Distance 2 (cm)      double   V7  Motor 1 state    string
    V8  Motor 2 state        string   V9  Servo 1 angle    integer
    V10 Servo 2 angle        integer  V11 Keypad status    string
*/

// ---- Blynk credentials: copy from your Blynk template/device (before the includes) ----
#define BLYNK_TEMPLATE_ID "TMPL61huFZ2Ok"
#define BLYNK_TEMPLATE_NAME "Esp32 Smart Home"
#define BLYNK_AUTH_TOKEN    "k-5bBYTQnykp4xyDxZnDr7xl9waKWTL2"

#define BLYNK_PRINT Serial      // Blynk debug messages in the Serial Monitor

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// =====================================================================
//  1. PIN DEFINITIONS
// =====================================================================
// Temperature (LM35) - ADC1
#define PIN_TEMP        32

// TB6612FNG
#define PIN_AIN1        16
#define PIN_AIN2        17
#define PIN_PWMA        26
#define PIN_BIN1        27
#define PIN_BIN2        4
#define PIN_PWMB        5
#define PIN_STBY        23

// Rain sensor (input-only ADC1)
#define PIN_RAIN        39

// MQ gas sensor (input-only ADC1) and buzzer
#define PIN_MQ          35
#define PIN_BUZZER      25

// Ultrasonic sensors
#define PIN_TRIG1       12      // strapping pin: keep LOW at boot (no pull-up!)
#define PIN_ECHO1       34      // input-only, use divider
#define PIN_TRIG2       13
#define PIN_ECHO2       33      // use divider

// Servos
#define PIN_SERVO1      2       // strapping pin: must be LOW/floating at boot
#define PIN_SERVO2      18

// I2C (default ESP32 DevKit pins)
#define PIN_SDA         21
#define PIN_SCL         22
#define KEYPAD_I2C_ADDR 0x20

// =====================================================================
//  2. CONFIGURABLE THRESHOLDS, TIMINGS, SPEEDS
// =====================================================================
// --- Temperature / Motor 1 ---
const float TEMP_THRESHOLD_C     = 35.0;   // Motor 1 runs at or above this
const float TEMP_HYSTERESIS_C    = 0.5;    // motor stops below (threshold - hysteresis). Set 0.0 for strict compare
const float TEMP_CAL_OFFSET_C    = 0.0;    // calibration offset vs. a reference thermometer
const uint16_t TEMP_SAMPLE_MS    = 500;

// --- Rain / Motor 2 ---
const int   RAIN_THRESHOLD       = 2000;   // raw ADC (0-4095)
const int   RAIN_HYSTERESIS      = 100;    // avoids flicker around the threshold
const bool  RAIN_WET_WHEN_LOWER  = true;   // true: value drops when wet. Set false if yours rises
const uint16_t RAIN_SAMPLE_MS    = 200;
const uint16_t MOTOR2_DEADTIME_MS = 500;   // stop time between direction changes
const uint32_t MOTOR2_RUN_MS      = 5000;  // Motor 2 runs this long after a threshold crossing, then stops

// --- Motor speeds (0-255) ---
const uint8_t MOTOR1_SPEED       = 200;
const uint8_t MOTOR2_SPEED       = 200;
const uint32_t MOTOR_PWM_FREQ    = 1000;   // Hz
const uint8_t  MOTOR_PWM_BITS    = 8;

// --- Gas / Buzzer ---
const int   GAS_THRESHOLD        = 2000;   // raw ADC (0-4095) - CALIBRATE in clean air first
const int   GAS_HYSTERESIS       = 100;
const uint32_t MQ_WARMUP_MS      = 60000;  // alarm disabled during warm-up (set 0 to test quickly)
const uint16_t GAS_SAMPLE_MS     = 200;
const uint16_t BUZZER_ON_MS      = 300;
const uint16_t BUZZER_OFF_MS     = 300;
const bool  BUZZER_ACTIVE_HIGH   = true;

// --- Ultrasonic / Servo 1 ---
const float DISTANCE_THRESHOLD_CM = 8.0;
const float DISTANCE_MIN_VALID_CM = 2.0;   // HC-SR04 cannot measure closer than ~2 cm
const float DISTANCE_MAX_VALID_CM = 100.0;
const uint32_t ECHO_TIMEOUT_US    = 6000;  // ~1 m round trip; no echo => invalid reading
const uint16_t ULTRASONIC_SLOT_MS = 60;    // one sensor per slot (avoids crosstalk)
const int   SERVO1_HOME_ANGLE     = 0;
const int   SERVO1_ACTIVE_ANGLE   = 90;
const bool  SERVO1_OSCILLATE      = false; // false: go to 90 while object present, 0 when clear
                                           // true : swing 0<->90 repeatedly while object present
const uint16_t SERVO1_SWEEP_MS    = 700;   // swing interval if SERVO1_OSCILLATE = true
const uint16_t SERVO1_RELEASE_MS  = 1000;  // object must be gone this long before returning to 0

// --- Keypad / Servo 2 ---
const char  KEYPAD_PASSWORD[]     = "123";
const uint8_t KEYPAD_MAX_LEN      = 8;
const bool  KEYPAD_AUTO_CONFIRM   = true;  // true: servo 2 runs as soon as "123" is typed (no '#' needed)
const uint16_t KEYPAD_SCAN_MS     = 20;    // two equal scans in a row = debounced key
const uint8_t KEYPAD_SETTLE_MS    = 2;     // row settle time (proven in keypad test sketch)
const bool  KEYPAD_PRINT_KEYS     = true;  // set false to hide key presses in Serial Monitor
const int   SERVO2_HOME_ANGLE     = 0;
const int   SERVO2_OPEN_ANGLE     = 90;
const uint32_t SERVO2_HOLD_MS     = 5000;  // time held at 90 deg

// --- WiFi / Blynk ---
char ssid[] = "Usitha's Phone";
char pass[] = "usitha123";
const uint16_t BLYNK_SEND_MS      = 1000;  // how often sensor values are pushed to the cloud
const uint32_t WIFI_RETRY_MS      = 10000; // retry WiFi this often while disconnected
const uint32_t BLYNK_RETRY_MS     = 5000;  // retry Blynk this often while disconnected

// --- Serial ---
const uint32_t SERIAL_BAUD        = 115200;
const uint16_t PRINT_INTERVAL_MS  = 1000;

// --- Keypad map and PCF8574 pin mapping (from working test sketch) ---
const char KEYMAP[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
const uint8_t KEYPAD_ROW_PINS[4] = {0, 1, 2, 3};   // R1..R4 -> P0..P3
const uint8_t KEYPAD_COL_PINS[4] = {4, 5, 6, 7};   // C1..C4 -> P4..P7

// =====================================================================
//  3. GLOBAL STATE
// =====================================================================
Servo servo1;
Servo servo2;
PCF8574 pcf8574(KEYPAD_I2C_ADDR);

enum MotorDir { DIR_STOP, DIR_FWD, DIR_REV };
MotorDir motor1Dir = DIR_STOP;
MotorDir motor2Dir = DIR_STOP;

// Temperature
float tempC = NAN;
bool  tempValid = false;

// Rain
int   rainRaw = 0;
bool  raining = false;
bool  rainInit = false;               // first reading only sets the baseline
bool  prevRaining = false;
bool  motor2Pending = false;          // waiting out dead-time before reversing
unsigned long motor2StopMs = 0;
bool  motor2Running = false;          // inside the 5 s run window
unsigned long motor2RunStartMs = 0;
MotorDir motor2Wanted = DIR_STOP;     // direction set by the last threshold crossing

// Gas
int   gasRaw = 0;
bool  gasValid = false;
bool  gasWarm = false;
bool  gasAlarm = false;
bool  buzzerOn = false;
unsigned long buzzerToggleMs = 0;

// Ultrasonic / Servo 1
float dist1 = -1.0, dist2 = -1.0;     // -1 = no valid echo
bool  objectNear = false;
bool  servo1Active = false;
int   servo1Angle = -1;               // -1 = unknown (forces first write)
unsigned long servo1LastNearMs = 0;
unsigned long servo1SweepMs = 0;
bool  ultrasonicTurn = false;         // alternates sensors

// Keypad / Servo 2
bool  keypadOK = false;
String enteredPwd = "";
String keypadStatus = "IDLE";
int   servo2Angle = -1;
bool  servo2Open = false;
unsigned long servo2OpenMs = 0;

// =====================================================================
//  4. LEDC PWM HELPERS (works on Arduino-ESP32 2.x and 3.x)
// =====================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  void motorPwmInit() {
    ledcAttach(PIN_PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttach(PIN_PWMB, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  }
  void pwmWriteA(uint8_t duty) { ledcWrite(PIN_PWMA, duty); }
  void pwmWriteB(uint8_t duty) { ledcWrite(PIN_PWMB, duty); }
#else
  // Channels 4 and 5 share LEDC timer 2, away from the servo channels (0,1)
  const int MOTOR_CH_A = 4;
  const int MOTOR_CH_B = 5;
  void motorPwmInit() {
    ledcSetup(MOTOR_CH_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(MOTOR_CH_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttachPin(PIN_PWMA, MOTOR_CH_A);
    ledcAttachPin(PIN_PWMB, MOTOR_CH_B);
  }
  void pwmWriteA(uint8_t duty) { ledcWrite(MOTOR_CH_A, duty); }
  void pwmWriteB(uint8_t duty) { ledcWrite(MOTOR_CH_B, duty); }
#endif

// =====================================================================
//  5. TB6612FNG MOTOR FUNCTIONS
//  The opposite direction pin is always cleared FIRST so both inputs
//  are never HIGH together (prevents conflicting commands / braking).
// =====================================================================
void motor1Forward() {
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_AIN1, HIGH);
  pwmWriteA(MOTOR1_SPEED);
  motor1Dir = DIR_FWD;
}

void motor1Stop() {
  pwmWriteA(0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  motor1Dir = DIR_STOP;
}

void motor2Forward() {
  digitalWrite(PIN_BIN2, LOW);
  digitalWrite(PIN_BIN1, HIGH);
  pwmWriteB(MOTOR2_SPEED);
  motor2Dir = DIR_FWD;
}

void motor2Reverse() {
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, HIGH);
  pwmWriteB(MOTOR2_SPEED);
  motor2Dir = DIR_REV;
}

void motor2Stop() {
  pwmWriteB(0);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  motor2Dir = DIR_STOP;
}

const char* dirName(MotorDir d) {
  switch (d) {
    case DIR_FWD: return "FORWARD";
    case DIR_REV: return "REVERSE";
    default:      return "STOP";
  }
}

// =====================================================================
//  6. TEMPERATURE (LM35) -> MOTOR 1
// =====================================================================
void updateTemperature() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < TEMP_SAMPLE_MS) return;
  last = now;

  // Average 16 calibrated millivolt readings (ESP32 ADC is noisy/non-linear)
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(PIN_TEMP);
  float mv = sum / 16.0;

  tempC = (mv / 10.0) + TEMP_CAL_OFFSET_C;           // LM35: 10 mV per deg C
  tempValid = (mv > 0.0) && (tempC > -10.0) && (tempC < 150.0);

  if (!tempValid) {                                  // invalid -> fail safe: stop
    if (motor1Dir != DIR_STOP) motor1Stop();
    return;
  }

  if (tempC >= TEMP_THRESHOLD_C) {
    if (motor1Dir != DIR_FWD) motor1Forward();
  } else if (tempC < (TEMP_THRESHOLD_C - TEMP_HYSTERESIS_C)) {
    if (motor1Dir != DIR_STOP) motor1Stop();
  }
}

// =====================================================================
//  7. RAIN SENSOR -> MOTOR 2
//  Motor runs ONLY after a threshold crossing, for MOTOR2_RUN_MS, then stops.
//  wet = forward, dry = reverse.
// =====================================================================
void applyMotor2(MotorDir d) {
  if (d == DIR_FWD)      motor2Forward();
  else if (d == DIR_REV) motor2Reverse();
  else                   motor2Stop();
}

void updateRain() {
  static unsigned long last = 0;
  unsigned long now = millis();

  // ---- sample + decide wet/dry (with hysteresis) ----
  if (now - last >= RAIN_SAMPLE_MS) {
    last = now;
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) sum += analogRead(PIN_RAIN);
    rainRaw = sum / 8;

    if (RAIN_WET_WHEN_LOWER) {
      if (rainRaw <= RAIN_THRESHOLD - RAIN_HYSTERESIS)      raining = true;
      else if (rainRaw >= RAIN_THRESHOLD + RAIN_HYSTERESIS) raining = false;
    } else {
      if (rainRaw >= RAIN_THRESHOLD + RAIN_HYSTERESIS)      raining = true;
      else if (rainRaw <= RAIN_THRESHOLD - RAIN_HYSTERESIS) raining = false;
    }

    // ---- threshold crossing detection ----
    if (!rainInit) {                                 // first reading: just remember state
      rainInit = true;
      prevRaining = raining;
    } else if (raining != prevRaining) {             // crossed the threshold
      prevRaining = raining;
      motor2Wanted = raining ? DIR_FWD : DIR_REV;    // wet = forward, dry = reverse

      if (motor2Running) {                           // was mid-run: stop, dead-time, then reverse
        motor2Stop();
        motor2Running = false;
        motor2StopMs = now;
        motor2Pending = true;
      } else if (!motor2Pending) {                   // idle: start the 5 s run now
        applyMotor2(motor2Wanted);
        motor2Running = true;
        motor2RunStartMs = now;
      }
      // if already in dead-time, motor2Wanted is simply updated
    }
  }

  // ---- dead-time finished -> start the 5 s run ----
  if (motor2Pending && (now - motor2StopMs >= MOTOR2_DEADTIME_MS)) {
    applyMotor2(motor2Wanted);
    motor2Running = true;
    motor2RunStartMs = now;
    motor2Pending = false;
  }

  // ---- 5 s elapsed -> motor OFF until the next crossing ----
  if (motor2Running && (now - motor2RunStartMs >= MOTOR2_RUN_MS)) {
    motor2Stop();
    motor2Running = false;
  }
}

// =====================================================================
//  8. MQ GAS SENSOR -> INTERMITTENT BUZZER
// =====================================================================
void buzzerWrite(bool on) {
  buzzerOn = on;
  digitalWrite(PIN_BUZZER, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
}

void updateGas() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < GAS_SAMPLE_MS) return;
  last = now;

  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(PIN_MQ);
  gasRaw = sum / 8;
  gasValid = (gasRaw > 0);                           // 0 = shorted/disconnected output
  gasWarm = (now >= MQ_WARMUP_MS);

  if (!gasWarm || !gasValid) {                       // no alarms during warm-up/fault
    gasAlarm = false;
    return;
  }
  if (gasRaw > GAS_THRESHOLD)                       gasAlarm = true;
  else if (gasRaw < GAS_THRESHOLD - GAS_HYSTERESIS) gasAlarm = false;
}

void updateBuzzer() {
  unsigned long now = millis();
  if (!gasAlarm) {
    if (buzzerOn) buzzerWrite(false);
    return;
  }
  // Intermittent beeping, non-blocking
  unsigned long interval = buzzerOn ? BUZZER_ON_MS : BUZZER_OFF_MS;
  if (now - buzzerToggleMs >= interval) {
    buzzerToggleMs = now;
    buzzerWrite(!buzzerOn);
  }
}

// =====================================================================
//  9. ULTRASONIC SENSORS -> SERVO 1
// =====================================================================
// Returns distance in cm, or -1.0 if no echo / out of valid range.
float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0;                    // timeout: echo missing
  float cm = duration * 0.0343f / 2.0f;
  if (cm < DISTANCE_MIN_VALID_CM || cm > DISTANCE_MAX_VALID_CM) return -1.0;
  return cm;
}

void moveServo1(int angle) {
  if (angle == servo1Angle) return;                  // avoid repeated moves
  servo1.write(angle);
  servo1Angle = angle;
}

void updateUltrasonic() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < ULTRASONIC_SLOT_MS) return;
  last = now;

  if (!ultrasonicTurn) dist1 = readUltrasonicCm(PIN_TRIG1, PIN_ECHO1);
  else                 dist2 = readUltrasonicCm(PIN_TRIG2, PIN_ECHO2);
  ultrasonicTurn = !ultrasonicTurn;

  bool n1 = (dist1 >= 0) && (dist1 <= DISTANCE_THRESHOLD_CM);
  bool n2 = (dist2 >= 0) && (dist2 <= DISTANCE_THRESHOLD_CM);
  objectNear = n1 || n2;                             // invalid readings count as "no object"
}

void updateServo1() {
  unsigned long now = millis();

  if (objectNear) {
    servo1LastNearMs = now;
    if (!servo1Active) {
      servo1Active = true;
      servo1SweepMs = now;
      moveServo1(SERVO1_ACTIVE_ANGLE);
    }
  } else if (servo1Active && (now - servo1LastNearMs >= SERVO1_RELEASE_MS)) {
    servo1Active = false;
  }

  if (servo1Active) {
    if (SERVO1_OSCILLATE) {
      if (now - servo1SweepMs >= SERVO1_SWEEP_MS) {
        servo1SweepMs = now;
        moveServo1(servo1Angle == SERVO1_ACTIVE_ANGLE ? SERVO1_HOME_ANGLE : SERVO1_ACTIVE_ANGLE);
      }
    } else {
      moveServo1(SERVO1_ACTIVE_ANGLE);
    }
  } else {
    moveServo1(SERVO1_HOME_ANGLE);                   // default position
  }
}

// =====================================================================
//  10. I2C KEYPAD (PCF8574) -> PASSWORD -> SERVO 2
//  Matrix scan (merged from the working keypad test sketch):
//   - set all pins HIGH, then pull ONE row LOW (P0..P3)
//   - wait for lines to settle, read the port
//   - a LOW bit on P4..P7 means that column is pressed
// =====================================================================
char scanKeypad() {
  for (uint8_t r = 0; r < 4; r++) {
    uint8_t out = 0xFF & ~(1 << KEYPAD_ROW_PINS[r]); // selected row low, rest high
    pcf8574.write8(out);
    delay(KEYPAD_SETTLE_MS);

    uint8_t in = pcf8574.read8();

    for (uint8_t c = 0; c < 4; c++) {
      if (!(in & (1 << KEYPAD_COL_PINS[c]))) {
        pcf8574.write8(0xFF);                        // release all lines
        return KEYMAP[r][c];
      }
    }
  }
  pcf8574.write8(0xFF);                              // no key: release all lines
  return 0;
}

void moveServo2(int angle) {
  if (angle == servo2Angle) return;
  servo2.write(angle);
  servo2Angle = angle;
}

String maskedEntry() {
  String s = "";
  for (uint8_t i = 0; i < enteredPwd.length(); i++) s += '*';
  return s;
}

void grantAccess() {
  keypadStatus = "ACCEPTED";
  moveServo2(SERVO2_OPEN_ANGLE);
  servo2Open = true;
  servo2OpenMs = millis();                           // (re)start 5 s hold timer
  enteredPwd = "";
}

void handleKey(char k) {
  if (KEYPAD_PRINT_KEYS) { Serial.print("[KEYPAD] key: "); Serial.println(k); }

  if (k >= '0' && k <= '9') {
    if (enteredPwd.length() < KEYPAD_MAX_LEN) enteredPwd += k;
    keypadStatus = "ENTERING";
    if (KEYPAD_AUTO_CONFIRM && enteredPwd == String(KEYPAD_PASSWORD)) {
      grantAccess();                                 // "123" typed -> servo runs immediately
    }
  } else if (k == '*') {                             // clear
    enteredPwd = "";
    keypadStatus = "CLEARED";
  } else if (k == '#') {                             // confirm (also works without auto-confirm)
    if (enteredPwd == String(KEYPAD_PASSWORD)) {
      grantAccess();
    } else {
      keypadStatus = "REJECTED";
    }
    enteredPwd = "";                                 // always clear after confirm
  } else {
    keypadStatus = "IGNORED KEY";                    // A-D unused
  }
}

void updateKeypad() {
  static unsigned long last = 0;
  static char lastRaw = 0;
  static char stableKey = 0;
  unsigned long now = millis();
  if (!keypadOK || now - last < KEYPAD_SCAN_MS) return;
  last = now;

  char k = scanKeypad();
  if (k == lastRaw) {                                // same result twice = debounced
    if (k != stableKey) {
      stableKey = k;
      if (k != 0) handleKey(k);                      // fire ONCE per press, not while held
    }
  }
  lastRaw = k;
}

void updateServo2() {
  if (servo2Open && (millis() - servo2OpenMs >= SERVO2_HOLD_MS)) {
    moveServo2(SERVO2_HOME_ANGLE);
    servo2Open = false;
    keypadStatus = "IDLE";
  }
}

// =====================================================================
//  11. BLYNK CLOUD (non-blocking: never stalls the control loop)
// =====================================================================
void updateBlynk() {
  static unsigned long lastWifi = 0, lastBlynk = 0, lastSend = 0;
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {               // WiFi down: retry in background
    if (now - lastWifi >= WIFI_RETRY_MS) {
      lastWifi = now;
      WiFi.disconnect();
      WiFi.begin(ssid, pass);
    }
    return;
  }

  if (!Blynk.connected()) {                          // WiFi up, cloud down: retry
    if (now - lastBlynk >= BLYNK_RETRY_MS) {
      lastBlynk = now;
      Blynk.connect(1500);                           // short timeout, max ~1.5 s block
    }
    return;
  }

  Blynk.run();

  if (now - lastSend < BLYNK_SEND_MS) return;
  lastSend = now;

  if (tempValid) Blynk.virtualWrite(V0, tempC);
  Blynk.virtualWrite(V1, rainRaw);
  Blynk.virtualWrite(V2, raining ? "WET" : "DRY");
  Blynk.virtualWrite(V3, gasRaw);
  Blynk.virtualWrite(V4, !gasValid ? "FAULT" : (!gasWarm ? "WARMING UP" : (gasAlarm ? "ALARM" : "OK")));
  Blynk.virtualWrite(V5, dist1);                     // -1 = no valid echo
  Blynk.virtualWrite(V6, dist2);
  Blynk.virtualWrite(V7, dirName(motor1Dir));
  Blynk.virtualWrite(V8, dirName(motor2Dir));
  Blynk.virtualWrite(V9, servo1Angle);
  Blynk.virtualWrite(V10, servo2Angle);
  Blynk.virtualWrite(V11, keypadStatus);
}

// =====================================================================
//  12. SERIAL STATUS
// =====================================================================
void printDist(const char* name, float d) {
  if (d < 0) Serial.printf("%s=none ", name);
  else       Serial.printf("%s=%.1fcm ", name, d);
}

void printStatus() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < PRINT_INTERVAL_MS) return;
  last = now;

  Serial.println("------------------------------------------------");
  if (tempValid)
    Serial.printf("[TEMP ] %.1f C | threshold=%.1f C | motor1=%s\n", tempC, TEMP_THRESHOLD_C, dirName(motor1Dir));
  else
    Serial.printf("[TEMP ] INVALID | threshold=%.1f C | motor1=%s\n", TEMP_THRESHOLD_C, dirName(motor1Dir));

  Serial.printf("[RAIN ] raw=%d | threshold=%d (wet when %s) | state=%s | motor2=%s%s\n",
                rainRaw, RAIN_THRESHOLD, RAIN_WET_WHEN_LOWER ? "lower" : "higher",
                raining ? "WET" : "DRY", dirName(motor2Dir), motor2Pending ? " (dead-time)" : "");

  if (!gasWarm)
    Serial.printf("[GAS  ] raw=%d | threshold=%d | WARMING UP (%lu s left) | alarm=OFF\n",
                  gasRaw, GAS_THRESHOLD, (MQ_WARMUP_MS - now) / 1000);
  else if (!gasValid)
    Serial.printf("[GAS  ] SENSOR FAULT (raw=0) | alarm=OFF\n");
  else
    Serial.printf("[GAS  ] raw=%d | threshold=%d | alarm=%s\n", gasRaw, GAS_THRESHOLD, gasAlarm ? "ON" : "OFF");

  Serial.print("[DIST ] ");
  printDist("d1", dist1);
  printDist("d2", dist2);
  Serial.printf("| threshold=%.1fcm | servo1=%d deg\n", DISTANCE_THRESHOLD_CM, servo1Angle);

  Serial.printf("[KEYPAD] %s | entry=%s | status=%s | servo2=%d deg\n",
                keypadOK ? "OK" : "NOT FOUND", maskedEntry().c_str(), keypadStatus.c_str(), servo2Angle);

  Serial.printf("[BLYNK] wifi=%s | cloud=%s\n",
                WiFi.status() == WL_CONNECTED ? "OK" : "DOWN", Blynk.connected() ? "OK" : "DOWN");
}

// =====================================================================
//  13. SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println("\nESP32 Smart Automation starting...");

  // --- Safe output states first ---
  pinMode(PIN_BUZZER, OUTPUT);  buzzerWrite(false);
  pinMode(PIN_TRIG1, OUTPUT);   digitalWrite(PIN_TRIG1, LOW);
  pinMode(PIN_TRIG2, OUTPUT);   digitalWrite(PIN_TRIG2, LOW);
  pinMode(PIN_ECHO1, INPUT);
  pinMode(PIN_ECHO2, INPUT);

  // --- ADC ---
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_TEMP, ADC_11db);
  analogSetPinAttenuation(PIN_RAIN, ADC_11db);
  analogSetPinAttenuation(PIN_MQ,   ADC_11db);

  // --- Servos (attach BEFORE motor PWM so they claim their LEDC channels first) ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo1.attach(PIN_SERVO1, 500, 2400);
  servo2.setPeriodHertz(50);
  servo2.attach(PIN_SERVO2, 500, 2400);
  moveServo1(SERVO1_HOME_ANGLE);
  moveServo2(SERVO2_HOME_ANGLE);

  // --- TB6612FNG ---
  pinMode(PIN_AIN1, OUTPUT);  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  motorPwmInit();
  motor1Stop();
  motor2Stop();
  digitalWrite(PIN_STBY, HIGH);                      // enable driver

  // --- I2C keypad ---
  Wire.begin(PIN_SDA, PIN_SCL);
  pcf8574.begin();
  if (pcf8574.isConnected()) {
    keypadOK = true;
    pcf8574.write8(0xFF);                            // release all lines HIGH
    Serial.printf("Keypad PCF8574 found at 0x%02X.\n", KEYPAD_I2C_ADDR);
  } else {
    Serial.printf("WARNING: PCF8574 NOT found at 0x%02X - check wiring/address.\n", KEYPAD_I2C_ADDR);
  }

  // --- WiFi + Blynk (non-blocking start; updateBlynk() handles connecting) ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN);

  Serial.printf("MQ warm-up: %lu s (alarm disabled until finished).\n", MQ_WARMUP_MS / 1000);
  Serial.println("Setup complete.");
}

void loop() {
  updateTemperature();   // LM35 -> Motor 1
  updateRain();          // rain threshold crossing -> Motor 2 runs 5 s
  updateGas();           // MQ    -> alarm state
  updateBuzzer();        // intermittent beeping
  updateUltrasonic();    // distances (one sensor per 60 ms slot)
  updateServo1();        // object within threshold -> Servo 1
  updateKeypad();        // scan + debounce + password logic
  updateServo2();        // return Servo 2 to 0 deg after hold time
  updateBlynk();         // push all sensor values to Blynk cloud
  printStatus();         // periodic Serial report
}
