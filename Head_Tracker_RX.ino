
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include "HeadTrackerProtocol.h"

// ---------- Settings ----------
const uint8_t CE_PIN   = 9;
const uint8_t CSN_PIN  = 10;
const uint8_t PAN_PIN  = 3;
const uint8_t TILT_PIN = 5;

// Head angle (degrees) that gives full servo travel either side of centre.
const float PAN_RANGE_DEG  = 90.0f;
const float TILT_RANGE_DEG = 90.0f;

// Flip a direction if the camera moves the wrong way.
// The original build is inverted on both axes.
const bool PAN_INVERT  = true;
const bool TILT_INVERT = true;

// Servo centre and travel limits in degrees (0-180). Reduce the limits to protect your mount.
const float PAN_CENTER  = 90, PAN_MIN  = 0, PAN_MAX  = 180;
const float TILT_CENTER = 90, TILT_MIN = 0, TILT_MAX = 180;

const float SMOOTHING          = 0.35f;  // 0-1: higher = faster, lower = smoother
const float DEADBAND_DEG       = 0.4f;   // ignore movements smaller than this (reduces jitter)
const unsigned long UPDATE_MS   = 20;    // servo update interval
const unsigned long FAILSAFE_MS = 500;   // no data for this long -> return to centre

const bool DEBUG_SERIAL = true;          // Serial Monitor at 115200 baud
const unsigned long DEBUG_INTERVAL_MS = 250;
// -------------------------------

// SG90 pulse range (Servo library defaults).
const int MIN_US = 544;
const int MAX_US = 2400;

RF24 radio(CE_PIN, CSN_PIN);
Servo panServo, tiltServo;
HeadPacket packet = {0, 0};

float targetPan = PAN_CENTER, targetTilt = TILT_CENTER;
float curPan = PAN_CENTER, curTilt = TILT_CENTER;
float lastPanWritten = -1000, lastTiltWritten = -1000;
unsigned long lastPacketMs = 0, lastUpdate = 0, lastDebug = 0;

static float headToServo(float headDeg, float range, bool invert, float center, float minA, float maxA) {
  headDeg = constrain(headDeg, -range, range);
  float a = headDeg * (90.0f / range);      // -90 ... +90
  if (invert) a = -a;
  return constrain(center + a, minA, maxA);
}

static int angleToUs(float angle) {
  return MIN_US + (int)(angle * (MAX_US - MIN_US) / 180.0f);
}

void setup() {
  Serial.begin(115200);

  if (!htConfigureRadio(radio, RF24_PA_LOW)) {
    Serial.println(F("nRF24L01 not responding - check wiring/power"));
    while (true) { delay(1000); }
  }
  radio.openReadingPipe(1, HT_ADDRESS);
  radio.startListening();

  panServo.attach(PAN_PIN, MIN_US, MAX_US);
  tiltServo.attach(TILT_PIN, MIN_US, MAX_US);
  panServo.writeMicroseconds(angleToUs(PAN_CENTER));
  tiltServo.writeMicroseconds(angleToUs(TILT_CENTER));
  Serial.println(F("Head tracker RX ready"));
}

void loop() {
  // Read every waiting packet and keep the newest, so old data never piles up.
  bool gotData = false;
  while (radio.available()) {
    radio.read(&packet, sizeof(packet));
    gotData = true;
  }
  if (gotData) {
    lastPacketMs = millis();
    targetPan  = headToServo(packet.pan,  PAN_RANGE_DEG,  PAN_INVERT,  PAN_CENTER,  PAN_MIN,  PAN_MAX);
    targetTilt = headToServo(packet.tilt, TILT_RANGE_DEG, TILT_INVERT, TILT_CENTER, TILT_MIN, TILT_MAX);
  }

  // Failsafe: if the link is lost, return to centre.
  if (millis() - lastPacketMs > FAILSAFE_MS) {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
  }

  // Smooth the movement and drive the servos at a fixed rate.
  if (millis() - lastUpdate >= UPDATE_MS) {
    lastUpdate = millis();
    curPan  += SMOOTHING * (targetPan  - curPan);
    curTilt += SMOOTHING * (targetTilt - curTilt);

    if (fabs(curPan - lastPanWritten) >= DEADBAND_DEG) {
      panServo.writeMicroseconds(angleToUs(curPan));
      lastPanWritten = curPan;
    }
    if (fabs(curTilt - lastTiltWritten) >= DEADBAND_DEG) {
      tiltServo.writeMicroseconds(angleToUs(curTilt));
      lastTiltWritten = curTilt;
    }
  }

  if (DEBUG_SERIAL && millis() - lastDebug >= DEBUG_INTERVAL_MS) {
    lastDebug = millis();
    Serial.print(F("pan: "));  Serial.print(curPan, 1);
    Serial.print(F("  tilt: ")); Serial.println(curTilt, 1);
  }
}
