// Mega 2560 line-follower skeleton: QTR-8A + vision lookahead packet + PID + optional TB6612FNG.

// ---------------- Pins (adjust as needed)
const int STBY = 8;
const int AIN1 = 7;
const int AIN2 = 6;
const int PWMA = 5;  // PWM
const int BIN1 = 4;
const int BIN2 = 3;
const int PWMB = 9;  // PWM

const int IR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ---------------- Feature state from Pi
float lastLk = 0.0f;       // 0..1
bool lastValid = false;
unsigned long lastVisionMs = 0;

// ---------------- Control loop
const unsigned long CONTROL_DT_US = 5000;  // 200 Hz
unsigned long lastControlUs = 0;

float Kp = 0.55f, Ki = 0.0f, Kd = 0.12f;
float integ = 0.0f;
float prevE = 0.0f;

float baseSpeed = 0.35f;  // normalized [-1..1], positive forward
const float BASE_MIN = 0.20f;
const float BASE_MAX = 0.70f;

// Set false if motors are not connected; commands will print instead.
const bool MOTOR_DRIVER_PRESENT = false;

// ---------------- Utility
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void readVisionSerial() {
  static char buf[40];
  static int idx = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      buf[idx] = '\0';
      idx = 0;
      if (buf[0] == 'L' && buf[1] == ',') {
        float lk = 0.0f;
        int v = 0;
        if (sscanf(buf, "L,%f,%d", &lk, &v) == 2) {
          lastLk = clampf(lk, 0.0f, 1.0f);
          lastValid = (v != 0);
          lastVisionMs = millis();
        }
      }
    } else {
      if (idx < (int)sizeof(buf) - 1) {
        buf[idx++] = c;
      } else {
        idx = 0;  // overflow reset
      }
    }
  }
}

bool computeIrError(float &e_out) {
  float sum = 0.0f;
  float wsum = 0.0f;
  for (int i = 0; i < 8; i++) {
    int raw = analogRead(IR_PINS[i]);  // 0..1023
    float v = raw / 1023.0f;
    // If your sensors see dark as low, invert: v = 1.0f - v;
    sum += v;
    wsum += v * i;
  }
  if (sum < 0.01f) return false;  // line lost
  float pos = wsum / sum;         // 0..7
  float center = 3.5f;
  float err_idx = pos - center;  // -3.5..+3.5
  const float spacing = 0.010f;  // 10 mm spacing assumption
  e_out = err_idx * spacing;
  return true;
}

void updateScheduler() {
  bool visionFresh = (millis() - lastVisionMs) < 200;
  bool useVision = visionFresh && lastValid;
  float lk = useVision ? lastLk : 0.0f;

  if (!useVision) {
    baseSpeed = 0.30f;
    Kp = 0.70f; Ki = 0.0f; Kd = 0.10f;
    return;
  }
  if (lk < 0.35f) {
    baseSpeed = 0.25f;
    Kp = 0.85f; Ki = 0.0f; Kd = 0.10f;
  } else if (lk < 0.70f) {
    baseSpeed = 0.40f;
    Kp = 0.60f; Ki = 0.0f; Kd = 0.12f;
  } else {
    baseSpeed = 0.60f;
    Kp = 0.45f; Ki = 0.0f; Kd = 0.16f;
  }
  baseSpeed = clampf(baseSpeed, BASE_MIN, BASE_MAX);
}

void setMotor(int IN1, int IN2, int PWM, float cmd) {
  cmd = clampf(cmd, -1.0f, 1.0f);
  int duty = (int)(fabs(cmd) * 255);
  if (cmd >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(PWM, duty);
}

void outputMotors(float left, float right) {
  if (!MOTOR_DRIVER_PRESENT) {
    Serial.print("CMD left=");
    Serial.print(left, 3);
    Serial.print(" right=");
    Serial.println(right, 3);
    return;
  }
  digitalWrite(STBY, HIGH);
  setMotor(AIN1, AIN2, PWMA, left);
  setMotor(BIN1, BIN2, PWMB, right);
}

void setup() {
  Serial.begin(115200);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void loop() {
  readVisionSerial();

  unsigned long nowUs = micros();
  if (nowUs - lastControlUs < CONTROL_DT_US) return;
  float dt = (nowUs - lastControlUs) / 1e6f;
  lastControlUs = nowUs;

  updateScheduler();

  float e;
  bool hasLine = computeIrError(e);

  float u = 0.0f;
  if (!hasLine) {
    baseSpeed *= 0.5f;
    integ = 0.0f;
    prevE = 0.0f;
    u = 0.0f;
  } else {
    integ += e * dt;
    float de = (e - prevE) / dt;
    prevE = e;
    u = Kp * e + Ki * integ + Kd * de;
  }

  float left = baseSpeed - u;
  float right = baseSpeed + u;
  left = clampf(left, -1.0f, 1.0f);
  right = clampf(right, -1.0f, 1.0f);

  outputMotors(left, right);
}
