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
double input, output;
double Kp = 2, Ki = 1, Kd = 1;
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

// ESC Control
Servo ESCNine, ESCTen;
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
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
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
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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
    myPID.SetMode(AUTOMATIC);

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
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[2] * 180 / M_PI;
        myPID.Compute();
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
        Serial.print("TEST: ");
        Serial.print(output);
        Serial.print("\t");
        Serial.print("Input: ");
        Serial.print(input);
        Serial.print("\t");
    }

    rpm9 += output;
    rpm10 -= output;

    if (rpm9 < 1250)
    {
        rpm9 = 1250;
    }

    if (rpm10 < 1250)
    {
        rpm10 = 1250;
    }

    if (rpm9 > 2000)
    {
        rpm9 = 2000;
    }

    if (rpm10 > 2000)
    {
        rpm10 = 2000;
    }

    ESCNine.writeMicroseconds(rpm9);
    ESCTen.writeMicroseconds(rpm10);
    // Serial.print("\n esc9: ");
    // Serial.print(rpm9);
    // Serial.print("\n esc10: ");
    // Serial.print(rpm10);
}