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
// Follow the DIAGNOSIS_GUIDE.md for instructions!
// ================================================================

// STEP 1: Calibrate your MPU6050 with an IMU_Zero sketch.
// Put the offset values you find here. These are just placeholders!
#define MPU_X_GYRO_OFFSET 220
#define MPU_Y_GYRO_OFFSET 76
#define MPU_Z_GYRO_OFFSET -85
#define MPU_Z_ACCEL_OFFSET 1788

// STEP 2 & 3: Find your base throttle and tune the PID "knobs".
int baseThrottle = 1150; // The basic idle speed for the motors (1000-2000). Find this value first.
double Kp_roll = 2.0;    // "P" for Proportional: The main reaction strength. Start here.
double Ki_roll = 0.0;    // "I" for Integral: Fixes small, long-term errors. Keep at 0 to start.
double Kd_roll = 0.0;    // "D" for Derivative: The "brakes" to prevent overshooting. Keep at 0 to start.

// ================================================================
// --- END OF USER TUNING AREA ---
// You shouldn't need to change anything below this line.
// ================================================================

// --- HARDWARE AND SENSOR SETUP ---
#define MOTOR_LEFT_PIN 9
#define MOTOR_RIGHT_PIN 10
#define INTERRUPT_PIN 2 // For MPU6050 on Arduino Mega, this is interrupt 0

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
float ypr[3]; // [yaw, pitch, roll]

// --- PID CONTROLLER SETUP ---
double setpointRoll = 0.0; // Our goal is to stay level (0 degrees roll)
double inputRoll, outputRoll;
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, Kp_roll, Ki_roll, Kd_roll, DIRECT);

// --- SAFETY & STATE ---
bool isArmed = false;
volatile bool mpuInterrupt = false; // "volatile" is important for variables used in interrupts
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
  Wire.setClock(400000); // 400kHz I2C clock
  Serial.begin(115200);

  // Attach motors to their pins and send a "safe" low signal to the ESCs
  motorLeft.attach(MOTOR_LEFT_PIN, 1000, 2000); // (pin, min pulse, max pulse)
  motorRight.attach(MOTOR_RIGHT_PIN, 1000, 2000);
  motorLeft.writeMicroseconds(1000);
  motorRight.writeMicroseconds(1000);
  delay(1000); // Wait for ESCs to initialize

  // Initialize MPU6050
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Load Motion Processing algorithms on the MPU6050
  devStatus = mpu.dmpInitialize();

  // Feed the MPU the calibration offsets from the Tuning Area above
  mpu.setXGyroOffset(MPU_X_GYRO_OFFSET);
  mpu.setYGyroOffset(MPU_Y_GYRO_OFFSET);
  mpu.setZGyroOffset(MPU_Z_GYRO_OFFSET);
  mpu.setZAccelOffset(MPU_Z_ACCEL_OFFSET);

  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP (Digital Motion Processor)..."));
    mpu.setDMPEnabled(true);

    // Set up the interrupt to tell us when new data is ready
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    // Configure the PID controller
    pidRoll.SetMode(AUTOMATIC);
    pidRoll.SetSampleTime(10);          // Check for updates every 10ms
    pidRoll.SetOutputLimits(-400, 400); // The PID can adjust the throttle up or down by 400

    Serial.println(F("\n======================================"));
    Serial.println(F("System Ready. Currently DISARMED."));
    Serial.println(F("Send 'a' via Serial Monitor to ARM."));
    Serial.println(F("Send any other character to DISARM."));
    Serial.println(F("======================================"));
  }
  else
  {
    // Error message if MPU setup fails
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
  // Check for arm/disarm commands from the Serial Monitor
  if (Serial.available() > 0)
  {
    char command = Serial.read();
    if (command == 'a' && !isArmed)
    {
      isArmed = true;
      Serial.println(F("ARMED. PID control is active. BE CAREFUL!"));
    }
    else if (isArmed)
    {
      isArmed = false;
      Serial.println(F("DISARMED. Motors stopped."));
    }
  }

  // If the system isn't ready or is disarmed, keep motors off and do nothing.
  if (!dmpReady || !isArmed)
  {
    motorLeft.writeMicroseconds(1000);
    motorRight.writeMicroseconds(1000);
    return; // Exit the loop early
  }

  // Wait for the MPU6050 to signal that it has new data
  if (!mpuInterrupt && fifoCount < packetSize)
  {
    return; // Not enough data yet, wait for next loop
  }

  // We have new data! Reset the interrupt flag.
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // Check for a data overflow (a bad thing)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    // Make sure we have a complete packet of data
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // Read the packet from the MPU
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Convert the packet into useful angles (yaw, pitch, and roll)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // --- The Magic Happens Here ---
    // 1. Get the current roll angle and feed it to the PID controller
    inputRoll = ypr[2] * 180 / M_PI; // We want roll (ypr[2]) in degrees
    pidRoll.Compute();               // The PID calculates how much correction is needed

    // 2. Apply the correction to the motors
    // If roll is positive (right side down), outputRoll will be negative.
    // motorLeft = base - (negative) = faster
    // motorRight = base + (negative) = slower
    // This pushes the left side up, correcting the roll.
    int motorSpeedLeft = baseThrottle - outputRoll;
    int motorSpeedRight = baseThrottle + outputRoll;

    // 3. Make sure the motor speeds are within safe limits (e.g., 1100 is a safe minimum spinning speed)
    motorSpeedLeft = constrain(motorSpeedLeft, 1100, 2000);
    motorSpeedRight = constrain(motorSpeedRight, 1100, 2000);

    // 4. Send the final commands to the motors
    motorLeft.writeMicroseconds(motorSpeedLeft);
    motorRight.writeMicroseconds(motorSpeedRight);

    // Optional: Print debugging info to the Serial Monitor
    // Serial.print("Roll(I): ");
    // Serial.print(inputRoll, 2); // Print with 2 decimal places
    // Serial.print("\t PID(O): ");
    // Serial.print(outputRoll, 2);
    // Serial.print("\t M_Left: ");
    // Serial.print(motorSpeedLeft);
    // Serial.print("\t M_Right: ");
    // Serial.println(motorSpeedRight);
  }
}