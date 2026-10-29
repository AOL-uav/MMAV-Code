/*
  Holds the rotational-mode AoA servos at their calibrated neutral positions.
  Left AoA: D4, 1575 us.  Right AoA: D3, 1500 us.
  No GPS, sweep, arming, or control logic is included.
*/

#include <Arduino.h>
#include <Servo.h>

static const uint8_t LEFT_AOA_PIN = 4;
static const uint8_t RIGHT_AOA_PIN = 3;
static const uint16_t LEFT_AOA_NEUTRAL_US = 1575;
static const uint16_t RIGHT_AOA_NEUTRAL_US = 1500;

static Servo leftAoa;
static Servo rightAoa;

void setup() {
  Serial.begin(115200);
  leftAoa.attach(LEFT_AOA_PIN, 500, 2500);
  rightAoa.attach(RIGHT_AOA_PIN, 500, 2500);
  leftAoa.writeMicroseconds(LEFT_AOA_NEUTRAL_US);
  rightAoa.writeMicroseconds(RIGHT_AOA_NEUTRAL_US);
  Serial.println(F("AoA servos holding neutral: left=1575 us, right=1500 us"));
}

void loop() {
  delay(1000);
}
