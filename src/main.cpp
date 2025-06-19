// --- LIBRARIES ---
#include <Arduino.h>
#include "PID.h"   // Our project's PID controller header
#include <Wire.h>  // For I2C communication with the sensor
#include <Servo.h> // For generating ESC control signals (it's the right tool!)

// MPU6050 libraries (managed by PlatformIO from lib_deps)
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ================================================================
// ===                 !! USER TUNING AREA !!                   ===
// ================================================================

// YOUR CALIBRATED NUMBERS
#define MPU_X_GYRO_OFFSET 2
#define MPU_Y_GYRO_OFFSET 2
#define MPU_Z_GYRO_OFFSET 40
#define MPU_Z_ACCEL_OFFSET 552

int baseThrottle = 1100;
double Kp_roll = 0.0;
double Ki_roll = 0.0;
double Kd_roll = 0.0;

// ================================================================
// --- END OF USER TUNING AREA ---
// ================================================================

// --- HARDWARE AND SENSOR SETUP ---
#define MOTOR_LEFT_PIN 9
#define MOTOR_RIGHT_PIN 10
#define INTERRUPT_PIN 2

Servo motorLeft;
Servo motorRight;
MPU6050 mpu;

// --- MPU6050 RUNTIME VARIABLES ---
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// --- PID CONTROLLER SETUP ---
double setpointRoll = 0.0;
double inputRoll, outputRoll;
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, Kp_roll, Ki_roll, Kd_roll, DIRECT);

// --- SAFETY & STATE ---
bool isArmed = false;
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  motorLeft.attach(MOTOR_LEFT_PIN, 1000, 2000);
  motorRight.attach(MOTOR_RIGHT_PIN, 1000, 2000);
  motorLeft.writeMicroseconds(1000);
  motorRight.writeMicroseconds(1000);
  delay(1000);

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(MPU_X_GYRO_OFFSET);
  mpu.setYGyroOffset(MPU_Y_GYRO_OFFSET);
  mpu.setZGyroOffset(MPU_Z_GYRO_OFFSET);
  mpu.setZAccelOffset(MPU_Z_ACCEL_OFFSET);

  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    pidRoll.SetMode(AUTOMATIC);
    pidRoll.SetSampleTime(10);
    pidRoll.SetOutputLimits(-400, 400);

    Serial.println(F("\n======================================"));
    Serial.println(F("FINAL DIAGNOSTIC. Ready to Arm."));
    Serial.println(F("Motors will NOT spin."));
    Serial.println(F("======================================"));
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  if (Serial.available() > 0)
  {
    char command = Serial.read();
    if (command == 'a' && !isArmed)
    {
      isArmed = true;
      mpu.resetFIFO();
      mpuInterrupt = false;
      Serial.println(F("ARMED. Checking for overflow..."));
    }
    else if (isArmed)
    {
      isArmed = false;
      Serial.println(F("DISARMED."));
    }
  }

  if (!dmpReady || !isArmed)
  {
    return;
  }

  if (!mpuInterrupt && fifoCount < packetSize)
  {
    return;
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!")); // We are watching this line.
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    inputRoll = ypr[2] * 180 / M_PI;
    pidRoll.Compute();

    int motorSpeedLeft = baseThrottle - outputRoll;
    int motorSpeedRight = baseThrottle + outputRoll;

    motorSpeedLeft = constrain(motorSpeedLeft, 1100, 2000);
    motorSpeedRight = constrain(motorSpeedRight, 1100, 2000);

    // --- MOTOR COMMANDS ARE DISABLED FOR THIS TEST ---
    // motorLeft.writeMicroseconds(motorSpeedLeft);
    // motorRight.writeMicroseconds(motorSpeedRight);
  }
}