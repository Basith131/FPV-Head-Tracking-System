/*
 * FPV Head Tracker - Transmitter (TX)
 * Board: Arduino Pro Mini (5 V, 16 MHz, ATmega328P)
 * Parts: MPU6050 motion sensor + nRF24L01 radio
 *
 * Based on the design and original sketches of Max Imagination
 * (tutorial: https://youtu.be/Tu0cay6S5WI), reworked with fixes
 * listed in the project README.
 *
 * Libraries: RF24 (TMRh20), MPU6050 (i2cdevlib by Jeff Rowberg)
 *
 * Wiring:
 *   MPU6050  VIN -> VCC, GND -> GND, SDA -> A4, SCL -> A5
 *   nRF24L01 CE -> D9, CSN -> D10 (on the 8-channel receiver board)
 *
 * Keep the unit still for a few seconds after power-on (sensor calibration).
 */
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "HeadTrackerProtocol.h"

// ---------- Settings ----------
const uint8_t  CE_PIN            = 9;
const uint8_t  CSN_PIN           = 10;
const unsigned long SEND_INTERVAL_MS = 20;   // 50 packets per second

// Which sensor angle drives which servo (index into ypr[]: 0 = yaw, 1 = pitch, 2 = roll).
// The original build uses the roll value for pan and the yaw value for tilt, because the
// MPU6050 is mounted standing up on the head. Swap these if your sensor is mounted differently.
const uint8_t  PAN_AXIS          = 2;
const uint8_t  TILT_AXIS         = 0;

// Optional re-centre button: connect a push button between this pin and GND.
// Set to -1 to disable. Pressing it makes the current head position the new "straight ahead".
const int8_t   RECENTER_PIN      = -1;

const bool     DEBUG_SERIAL      = true;     // print values to the Serial Monitor (115200 baud)
const unsigned long DEBUG_INTERVAL_MS = 250;
// -------------------------------

MPU6050 mpu;
RF24 radio(CE_PIN, CSN_PIN);

uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

HeadPacket packet = {0, 0};
float panOffset = 0, tiltOffset = 0;      // re-centre offsets (degrees)
float rawPan = 0, rawTilt = 0;
bool havePacket = false;
unsigned long lastSend = 0, lastDebug = 0, lastButton = 0;

static float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static void halt(const __FlashStringHelper *msg) {
  Serial.println(msg);
  while (true) { delay(1000); }
}

void setup() {
  Serial.begin(115200);

  // --- MPU6050 ---
  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  if (!mpu.testConnection()) halt(F("MPU6050 not found - check VIN/GND/SDA/SCL wiring"));

  if (mpu.dmpInitialize() != 0) halt(F("DMP initialization failed"));
  Serial.println(F("Calibrating - keep the unit still..."));
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);

  // --- nRF24L01 ---
  if (!htConfigureRadio(radio, RF24_PA_LOW)) halt(F("nRF24L01 not responding - check wiring/power"));
  radio.openWritingPipe(HT_ADDRESS);
  radio.stopListening();

  if (RECENTER_PIN >= 0) pinMode(RECENTER_PIN, INPUT_PULLUP);
  Serial.println(F("Head tracker TX ready"));
}

void loop() {
  // Read the newest orientation from the sensor's DMP.
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    rawPan  = ypr[PAN_AXIS]  * 180.0f / M_PI;
    rawTilt = ypr[TILT_AXIS] * 180.0f / M_PI;
    packet.pan  = wrap180(rawPan  - panOffset);
    packet.tilt = wrap180(rawTilt - tiltOffset);
    havePacket = true;
  }

  // Optional re-centre button (with simple debounce).
  if (RECENTER_PIN >= 0 && digitalRead(RECENTER_PIN) == LOW && millis() - lastButton > 400) {
    lastButton = millis();
    panOffset  = rawPan;
    tiltOffset = rawTilt;
  }

  // Send at a fixed rate.
  if (havePacket && millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    radio.write(&packet, sizeof(packet));
  }

  if (DEBUG_SERIAL && millis() - lastDebug >= DEBUG_INTERVAL_MS) {
    lastDebug = millis();
    Serial.print(F("pan: "));  Serial.print(packet.pan, 1);
    Serial.print(F("  tilt: ")); Serial.println(packet.tilt, 1);
  }
}
