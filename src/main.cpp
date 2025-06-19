// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

// #include "MPU6050.h" // not necessary if using MotionApps include file
#include "MPU6050_6Axis_MotionApps20.h"

// Include PID Controller library
#include "PID.h"

// Include Servo library
#include "Servo.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;


#define INTERRUPT_PIN 2

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// PID Control
double setPoint = 0;
double input = 0, output = 0;
double Kp = 2.2, Ki = 1.3, Kd = 1;
PID myPID(setPoint, Kp, Ki, Kd);

// ESC Control
Servo ESCNine, ESCTen;
int baseThrottle = 1200;
int rpm9 = 1000;
int rpm10 = 1000;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
    // MPU setup
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Wire.setWireTimeout(3000, true);

    // initialize serial communication
    Serial.begin(115200);
    Serial.setTimeout(10);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    while (Serial.available() && Serial.read())
        ; // empty buffer
    while (!Serial.available())
        ; // wait for data
    while (Serial.available() && Serial.read())
        ; // empty buffer again
    mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-1398); // <-- YOUR X-ACCEL OFFSET
    mpu.setYAccelOffset(1580);  // <-- YOUR Y-ACCEL OFFSET
    mpu.setZAccelOffset(562);   // <-- YOUR Z-ACCEL OFFSET
    mpu.setXGyroOffset(-1);     // <-- YOUR X-GYRO OFFSET
    mpu.setYGyroOffset(5);      // <-- YOUR Y-GYRO OFFSET
    mpu.setZGyroOffset(41);     // <-- YOUR Z-GYRO OFFSET

    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // PID Control setup
    input = ypr[2] * 180 / M_PI;

    ESCNine.attach(9, 1000, 2000);
    ESCTen.attach(10, 1000, 2000);

    ESCNine.writeMicroseconds(rpm9);
    ESCTen.writeMicroseconds(rpm10);
    delay(7000);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        // Grab angle using gyrometer in degrees
        double gyroAngle = ypr[2] * 180 / M_PI;

        // Grab angle using accelerometer in degrees
        float realAccX = (float)aa.x/16384;
        float realAccY = (float)aa.y/16384;
        float realAccZ = (float)aa.z/16384;
        float accelerometerAngle = -atan(realAccX/sqrt(realAccY*realAccY + realAccZ*realAccZ)) * M_PI/180;


        // Complementary filter on the angles
        input = 0.96*gyroAngle + 0.04*(double)accelerometerAngle;
        
        if(!myPID.computeOutput(input, &output)){
          myPID.prevTime_ = millis();
          return;
        }
        Serial.print("Roll Gyro Angle\t");
        // Serial.print(ypr[0] * 180 / M_PI);
        // Serial.print("\t");
        // Serial.print(ypr[1] * 180 / M_PI);
        // Serial.print("\t");
        Serial.print(gyroAngle);
        Serial.print("\t");
        Serial.print("Accelerometer: \t");
        Serial.print(accelerometerAngle);
        Serial.print("\t");
        Serial.print("Input:");
        Serial.print(input);
        Serial.println();
        // Serial.print("TEST: ");
        // Serial.print(output);
        // Serial.print("\t");
        // Serial.print("Input: ");
        // Serial.print(input);
        // Serial.print("\t");
    }

    rpm9 = baseThrottle + output;
    rpm10 = baseThrottle - output;

    rpm9 = constrain(rpm9, 1100, 1400);
    rpm10 = constrain(rpm10, 1100, 1400);

    ESCNine.writeMicroseconds(rpm9);
    ESCTen.writeMicroseconds(rpm10);
    // Serial.print("\n esc9: ");
    // Serial.print(rpm9);
    // Serial.print("\n esc10: ");
    // Serial.print(rpm10);
}