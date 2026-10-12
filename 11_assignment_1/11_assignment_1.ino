#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

// Raise toward 1 to follow motion faster. Lower toward 0 to reduce jitter.
#define _EMA_ALPHA 0.6    // EMA weight of new sample (range: 0 to 1)

// MG90S pulse width calibrated in 11_example_1. 500~2400 us is treated as 0~180 deg.
#define _DUTY_MIN 500
#define _DUTY_MAX 2400

// global variables
unsigned long last_sampling_time;  // unit: msec
float dist_prev = _DIST_MAX;       // last in-range distance (unit: mm)
float dist_ema = _DIST_MAX;        // EMA distance (unit: mm)
int duty;                          // commanded pulse width (unit: us)

Servo myservo;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, 1);  // LED OFF until a valid reading arrives

  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);
  duty = _DUTY_MAX;          // dist_ema starts at _DIST_MAX => 180 deg
  myservo.writeMicroseconds(duty);

  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered, angle;

  if (millis() < last_sampling_time + INTERVAL)
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // range filter: keep the previous in-range sample when the reading fails
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_filtered = dist_prev;
    digitalWrite(PIN_LED, 1);          // LED OFF
  } else if (dist_raw < _DIST_MIN) {
    dist_filtered = dist_prev;
    digitalWrite(PIN_LED, 1);          // LED OFF
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, 0);          // LED ON
  }

  dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

  // 18 cm -> 0 deg, 36 cm -> 180 deg, linear in between
  if (dist_ema <= _DIST_MIN) {
    angle = 0.0;
  } else if (dist_ema >= _DIST_MAX) {
    angle = 180.0;
  } else {
    angle = (dist_ema - _DIST_MIN) * 180.0 / (_DIST_MAX - _DIST_MIN);
  }

  duty = (int)(_DUTY_MIN + (angle / 180.0) * (_DUTY_MAX - _DUTY_MIN) + 0.5);
  myservo.writeMicroseconds(duty);

  Serial.print("Min:");       Serial.print(_DIST_MIN);
  Serial.print(",raw:");      Serial.print(dist_raw);
  Serial.print(",filtered:"); Serial.print(dist_filtered);
  Serial.print(",ema:");      Serial.print(dist_ema);
  Serial.print(",angle:");    Serial.print(angle);
  Serial.print(",duty:");     Serial.print(duty);
  Serial.print(",Max:");      Serial.print(_DIST_MAX);
  Serial.println("");

  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
