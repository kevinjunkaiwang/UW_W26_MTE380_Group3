// Arduino Mega 2560 motor I/O coprocessor for Pi-driven control.
// Role:
// - Receive wheel commands from Raspberry Pi: C,seq,ms,left,right,ttl_ms
// - Apply PWM to TB6612FNG
// - Read IR array and stream telemetry to Pi:
//   T,seq,ms,ir0,ir1,ir2,ir3,ir4,ir5,ir6,ir7,left_applied,right_applied,flags
//
// Design intent:
// - Arduino handles deterministic low-level timing (PWM + sensor sampling).
// - Raspberry Pi handles higher-level control logic (vision + fuzzy + PID).
// - Watchdog logic on Arduino guarantees safe stop if commands become stale.

#include <stdlib.h>
#include <string.h>
#include <math.h>

// ---------------- Serial
const unsigned long SERIAL_BAUD = 230400;
const unsigned long TELEMETRY_DT_MS = 10;  // 100 Hz telemetry

// ---------------- Control timing
const unsigned long CONTROL_DT_US = 5000;      // 200 Hz motor update
const unsigned long MAX_CMD_STALE_MS = 500;    // hard stale guard
const uint16_t DEFAULT_CMD_TTL_MS = 120;       // if command ttl is invalid

// Set false for bench testing without motor driver connected.
const bool MOTOR_DRIVER_PRESENT = true;

// ---------------- TB6612FNG pins
const int STBY = 8;
const int AIN1 = 7;
const int AIN2 = 6;
const int PWMA = 5;  // PWM
const int BIN1 = 4;
const int BIN2 = 3;
const int PWMB = 9;  // PWM

// ---------------- IR pins
const int IR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int IR_MIN_SUM = 16;  // simple line-present threshold for telemetry flag

// ---------------- Command state from Pi
volatile bool hasCommand = false;
unsigned long lastCmdRxMs = 0;
unsigned long lastCmdSeq = 0;
unsigned long lastCmdHostMs = 0;
uint16_t cmdTtlMs = DEFAULT_CMD_TTL_MS;
float cmdLeft = 0.0f;   // [-1,1]
float cmdRight = 0.0f;  // [-1,1]

// ---------------- Applied output + telemetry state
float appliedLeft = 0.0f;
float appliedRight = 0.0f;
int irRaw[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long telemetrySeq = 0;

unsigned long lastControlUs = 0;
unsigned long lastTelemetryMs = 0;

float clampf(float x, float lo, float hi) {
  // Lightweight clamp helper for command/signal bounds.
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void setMotor(int IN1, int IN2, int PWM, float cmd) {
  // Convert normalized command [-1,1] to TB6612FNG direction + PWM duty.
  cmd = clampf(cmd, -1.0f, 1.0f);
  int duty = (int)(fabs(cmd) * 255.0f);
  if (cmd >= 0.0f) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(PWM, duty);
}

void outputMotors(float left, float right) {
  // Store applied values for telemetry and drive motors if hardware is present.
  left = clampf(left, -1.0f, 1.0f);
  right = clampf(right, -1.0f, 1.0f);
  appliedLeft = left;
  appliedRight = right;

  if (!MOTOR_DRIVER_PRESENT) {
    return;
  }

  digitalWrite(STBY, HIGH);
  setMotor(AIN1, AIN2, PWMA, left);
  setMotor(BIN1, BIN2, PWMB, right);
}

void stopMotors() {
  // Immediate stop command used by stale-command watchdog and startup state.
  appliedLeft = 0.0f;
  appliedRight = 0.0f;
  if (!MOTOR_DRIVER_PRESENT) {
    return;
  }

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(STBY, LOW);
}

void readIrArray() {
  // Sample all IR channels each control tick.
  for (int i = 0; i < 8; i++) {
    irRaw[i] = analogRead(IR_PINS[i]);
  }
}

bool irLinePresent() {
  // Coarse line-present bit for telemetry/diagnostics.
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += irRaw[i];
  }
  return sum > IR_MIN_SUM;
}

bool isCommandFresh() {
  // Command is valid only if a recent packet exists and TTL has not expired.
  if (!hasCommand) return false;
  unsigned long age = millis() - lastCmdRxMs;
  if (age > MAX_CMD_STALE_MS) return false;
  if (age > cmdTtlMs) return false;
  return true;
}

bool parseCommandLine(char* line) {
  // Expected: C,seq,ms,left,right,ttl_ms
  // Returns true on successful parse and state update.
  char* save = NULL;
  char* tok = strtok_r(line, ",", &save);
  if (!tok || tok[0] != 'C') return false;

  tok = strtok_r(NULL, ",", &save);
  if (!tok) return false;
  unsigned long seq = strtoul(tok, NULL, 10);

  tok = strtok_r(NULL, ",", &save);
  if (!tok) return false;
  unsigned long hostMs = strtoul(tok, NULL, 10);

  tok = strtok_r(NULL, ",", &save);
  if (!tok) return false;
  float left = (float)atof(tok);

  tok = strtok_r(NULL, ",", &save);
  if (!tok) return false;
  float right = (float)atof(tok);

  tok = strtok_r(NULL, ",", &save);
  uint16_t ttl = DEFAULT_CMD_TTL_MS;
  if (tok) {
    unsigned long rawTtl = strtoul(tok, NULL, 10);
    if (rawTtl >= 20 && rawTtl <= 1000) {
      ttl = (uint16_t)rawTtl;
    }
  }

  cmdLeft = clampf(left, -1.0f, 1.0f);
  cmdRight = clampf(right, -1.0f, 1.0f);
  cmdTtlMs = ttl;
  lastCmdSeq = seq;
  lastCmdHostMs = hostMs;
  lastCmdRxMs = millis();
  hasCommand = true;
  return true;
}

void readCommandSerial() {
  // Read ASCII stream and parse newline-terminated command packets.
  static char buf[96];
  static int idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      buf[idx] = '\0';
      idx = 0;
      if (buf[0] != '\0') {
        parseCommandLine(buf);
      }
      continue;
    }
    if (idx < (int)sizeof(buf) - 1) {
      buf[idx++] = c;
    } else {
      idx = 0;  // overflow protection
    }
  }
}

void sendTelemetry() {
  // flags bitmask:
  // bit0 (0x01): command fresh
  // bit1 (0x02): line present by IR sum heuristic
  unsigned int flags = 0;
  if (isCommandFresh()) flags |= 0x01;
  if (irLinePresent()) flags |= 0x02;

  Serial.print("T,");
  Serial.print(telemetrySeq++);
  Serial.print(",");
  Serial.print(millis());

  for (int i = 0; i < 8; i++) {
    Serial.print(",");
    Serial.print(irRaw[i]);
  }

  Serial.print(",");
  Serial.print(appliedLeft, 3);
  Serial.print(",");
  Serial.print(appliedRight, 3);
  Serial.print(",");
  Serial.println(flags);
}

void setup() {
  // Initialize serial, pins, and safe startup state.
  Serial.begin(SERIAL_BAUD);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  stopMotors();
  readIrArray();
  lastControlUs = micros();
  lastTelemetryMs = millis();
}

void loop() {
  // 1) Process inbound command packets.
  readCommandSerial();

  // 2) Fixed-rate motor/sensor update loop.
  unsigned long nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_DT_US) {
    lastControlUs = nowUs;
    readIrArray();

    if (isCommandFresh()) {
      outputMotors(cmdLeft, cmdRight);
    } else {
      stopMotors();
    }
  }

  // 3) Fixed-rate telemetry stream to Pi.
  unsigned long nowMs = millis();
  if (nowMs - lastTelemetryMs >= TELEMETRY_DT_MS) {
    lastTelemetryMs = nowMs;
    sendTelemetry();
  }
}
