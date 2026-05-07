#include <Wire.h>
#include <PWMServo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ─── Pin assignments ───
const int sensorPin = A0;            // phototransistor analog read
const int ledPin    = 6;             // IR LED for turbidity
const int servoPin1 = SERVO_PIN_A;   // pin 9 on Uno
const int servoPin2 = SERVO_PIN_B;   // pin 10 on Uno
const int pumpPin   = 7;             // peristaltic pump relay
const int trigPin   = 5;             // ultrasonic TRIG
const int echoPin   = 3;             // ultrasonic ECHO
const int buttonPin = A3;            // OLED FeatherWing Button A, active LOW

// ─── Sampling ───
const int N_PAIRS = 50;

// ─── Tunable thresholds ───
// Signal = LED-on voltage minus LED-off voltage.
// More negative usually means more light transmitted / clearer.
// Less negative usually means less light transmitted / dirtier.

// Air/no-water detection only.
// If signal is below this, display "NO WATER" and return -1%.
const double AIR_CUTOFF = -0.3;

// Custom turbidity percent mapping only.
// MAX_CLEAN_SIGNAL maps to 0% turbidity.
// MAX_DIRTY_SIGNAL maps to 100% turbidity.
//
// Example:
// clean water signal = -0.18  -> 0%
// dirty water signal = -0.09  -> 100%
const double MAX_CLEAN_SIGNAL = -0.2;
const double MAX_DIRTY_SIGNAL = -0.09;

// Below this mapped percent, water is treated as clean.
const double CLEAN_PCT = 20.0;

// ─── Overflow ───
const double OVERFLOW_CM = 3.0;      // distance below this = overflow

// ─── Timing ───
const int  COUNT_NEED                = 10;
const unsigned long SWAP_DELAY_MS     = 300;
const unsigned long DISPENSE_DELAY_MS = 3000;
const unsigned long DEBOUNCE_MS       = 50;
const unsigned long DEBUG_PERIOD_MS   = 1000;

// ─── OLED ───
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// ─── Servos ───
PWMServo servo1;
PWMServo servo2;

// ─── State machine ───
enum SystemState {
  STATE_IDLE,
  STATE_RUNNING,
  STATE_DISPENSED,
  STATE_OVERFLOW
};

SystemState sysState = STATE_IDLE;

// isDirty means routing/state:
// true  = dirty / recirculation route
// false = clean route
bool isDirty         = true;
int  transitionCount = 0;
bool seenDirty       = true;

// Button edge detection
int lastButtonReading = HIGH;
int buttonState       = HIGH;
unsigned long lastDebounceTime = 0;

// ─── Helpers ───

static inline double adcToV(double adc) {
  return 5.0 * adc / 1023.0;
}

static inline int readADCSettled() {
  (void)analogRead(sensorPin);
  return analogRead(sensorPin);
}

void servosClean() {
  servo2.write(175);
  servo1.write(90);
}

void servosDirty() {
  servo2.write(90);
  servo1.write(15);
}

void pumpOff() {
  digitalWrite(pumpPin, HIGH);   // relay HIGH = off
}

void pumpOn() {
  digitalWrite(pumpPin, LOW);    // relay LOW = on
}

double readDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return 0.0;  // no echo / out of range
  }

  return duration * 0.0343 / 2.0;
}

double signalToPercent(double sig_avg_v) {
  // Air is handled separately from turbidity mapping.
  if (sig_avg_v < AIR_CUTOFF) return -1.0;

  // Custom linear mapping:
  // MAX_CLEAN_SIGNAL -> 0%
  // MAX_DIRTY_SIGNAL -> 100%
  double pct = 100.0 * (sig_avg_v - MAX_CLEAN_SIGNAL) /
               (MAX_DIRTY_SIGNAL - MAX_CLEAN_SIGNAL);

  // Clamp percent to 0–100.
  if (pct < 0.0)   pct = 0.0;
  if (pct > 100.0) pct = 100.0;

  return pct;
}

bool buttonPressed() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  bool pressedEdge = false;

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        pressedEdge = true;
      }
    }
  }

  lastButtonReading = reading;
  return pressedEdge;
}

// ─── Display ───

void drawIdleScreen() {
  display.clearDisplay();
  display.setRotation(1);
  display.setTextColor(SH110X_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Adaptive Purifier");
  display.drawLine(0, 10, 127, 10, SH110X_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 24);
  display.println("Press A");
  display.setCursor(0, 44);
  display.println("to Start");

  display.display();
}

void drawRunningScreen(double sig_avg_v, double turbidityPct, bool dirtyState, double dist_cm) {
  display.clearDisplay();
  display.setRotation(1);
  display.setTextColor(SH110X_WHITE);

  int sig_mv = (int)(sig_avg_v * 1000.0);
  int dist_mm = (int)(dist_cm * 10.0);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Turbidity");
  display.drawLine(0, 10, 127, 10, SH110X_WHITE);

  if (turbidityPct < 0) {
    display.setTextSize(2);
    display.setCursor(0, 18);
    display.println("NO");
    display.setCursor(0, 36);
    display.println("WATER");
  } else {
    display.setTextSize(3);
    display.setCursor(0, 16);
    display.print((int)(turbidityPct + 0.5));
    display.print("%");
  }

  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("State:");
  display.print(dirtyState ? "DIRTY" : "CLEAN");

  display.setCursor(0, 56);
  display.print("Sig:");
  display.print(sig_mv);
  display.print("mV");

  display.setCursor(70, 56);
  display.print("D:");
  display.print(dist_mm);
  display.print("mm");

  display.display();
}

void drawDispensedScreen() {
  display.clearDisplay();
  display.setRotation(1);
  display.setTextColor(SH110X_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Status");
  display.drawLine(0, 10, 127, 10, SH110X_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 18);
  display.println("Clean");
  display.setCursor(0, 36);
  display.println("Water");
  display.setCursor(0, 54);
  display.println("Out!");

  display.display();
}

void drawOverflowScreen() {
  display.clearDisplay();
  display.setRotation(1);
  display.setTextColor(SH110X_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("!! WARNING !!");
  display.drawLine(0, 10, 127, 10, SH110X_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 18);
  display.println("OVER");
  display.setCursor(0, 36);
  display.println("FLOW");

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.println("Press A to clear");

  display.display();
}

void resetRun() {
  isDirty         = true;
  seenDirty       = false;
  transitionCount = 0;

  servosDirty();
  pumpOff();
}

// ─── Setup ───

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F(">>> BOOT <<<"));

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(pumpPin, OUTPUT);
  pumpOff();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  if (!display.begin(0x3C, true)) {
    Serial.println(F("OLED not found"));
    while (1);
  }

  Serial.println(F("OLED OK"));

  delay(100);
  display.clearDisplay();
  display.display();

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servosDirty();

  drawIdleScreen();
  Serial.println(F("Ready."));
}

// ─── Main loop ───

void loop() {
  bool pressed = buttonPressed();

  switch (sysState) {

    case STATE_IDLE: {
      drawIdleScreen();

      if (pressed) {
        resetRun();
        pumpOn();
        sysState = STATE_RUNNING;
        Serial.println(F("START -> RUNNING"));
      }

      break;
    }

    case STATE_RUNNING: {
      if (pressed) {
        pumpOff();
        servosClean();
        sysState = STATE_IDLE;
        drawIdleScreen();
        Serial.println(F("STOP -> IDLE"));
        break;
      }

      // ─── Overflow check ───
      double dist_cm = readDistanceCM();

      if (dist_cm > 0.0 && dist_cm < OVERFLOW_CM) {
        pumpOff();
        sysState = STATE_OVERFLOW;
        drawOverflowScreen();
        Serial.println(F("OVERFLOW"));
        break;
      }

      // ─── Turbidity sampling ───
      long sumSig = 0;

      for (int i = 0; i < N_PAIRS; i++) {
        digitalWrite(ledPin, LOW);
        int off = readADCSettled();

        digitalWrite(ledPin, HIGH);
        int on = readADCSettled();

        sumSig += (on - off);
      }

      digitalWrite(ledPin, LOW);

      double sig_avg_v    = adcToV((double)sumSig / N_PAIRS);
      double turbidityPct = signalToPercent(sig_avg_v);

      bool isAir   = (turbidityPct < 0);
      bool isClean = (!isAir && turbidityPct < CLEAN_PCT);

      // ─── State logic ───
      // Dispense complete when we go CLEAN -> NO WATER, having seen dirty.
      if (isAir) {
        if (!isDirty && seenDirty) {
          delay(DISPENSE_DELAY_MS);
          pumpOff();
          servosClean();
          sysState = STATE_DISPENSED;
          drawDispensedScreen();
          Serial.println(F("DISPENSED"));
          break;
        }

        transitionCount = 0;
      }

      else if (!isClean) {
        if (!isDirty) {
          transitionCount++;

          if (transitionCount > COUNT_NEED) {
            delay(SWAP_DELAY_MS);

            isDirty   = true;
            seenDirty = true;

            servosDirty();
            transitionCount = 0;

            Serial.println(F("SWITCH -> DIRTY"));
          }
        } else {
          transitionCount = 0;
        }
      }

      else {
        if (isDirty) {
          transitionCount++;

          if (transitionCount > COUNT_NEED) {
            delay(SWAP_DELAY_MS);

            isDirty = false;

            servosClean();
            transitionCount = 0;

            Serial.println(F("SWITCH -> CLEAN"));
          }
        } else {
          transitionCount = 0;
        }
      }

      // ─── Debug, throttled and integer-only ───
      static unsigned long lastDbg = 0;

      if (millis() - lastDbg > DEBUG_PERIOD_MS) {
        int sig_mv = (int)(sig_avg_v * 1000.0);
        int turb_int = (turbidityPct < 0) ? -1 : (int)(turbidityPct + 0.5);
        int dist_mm = (int)(dist_cm * 10.0);

        Serial.print(F("sig="));
        Serial.print(sig_mv);
        Serial.print(F("mV turb="));
        Serial.print(turb_int);
        Serial.print(F("% dirty="));
        Serial.print(isDirty);
        Serial.print(F(" seen="));
        Serial.print(seenDirty);
        Serial.print(F(" count="));
        Serial.print(transitionCount);
        Serial.print(F(" dist="));
        Serial.print(dist_mm);
        Serial.println(F("mm"));

        lastDbg = millis();
      }

      drawRunningScreen(sig_avg_v, turbidityPct, isDirty, dist_cm);

      delay(50);
      break;
    }

    case STATE_DISPENSED: {
      if (pressed) {
        resetRun();
        sysState = STATE_IDLE;
        drawIdleScreen();
        Serial.println(F("DISPENSED -> IDLE"));
      }

      break;
    }

    case STATE_OVERFLOW: {
      if (pressed) {
        resetRun();
        sysState = STATE_IDLE;
        drawIdleScreen();
        Serial.println(F("OVERFLOW -> IDLE"));
      }

      break;
    }
  }
}