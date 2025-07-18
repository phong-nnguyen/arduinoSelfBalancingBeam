# Self Balancing Beam utilizing Arduino Mega 2560

The setup involves two brushless motors attached to a beam and an MPU6050 places in the middle of the beam. Motor speeds are controlled utilizing a PID control loop that takes in gyroscope data to get the current angle. However, gyroscope data tends to drift in the long run, which is why a sensor-fusion between gyroscope and accelerometer was done. Accelerometer data is good in the long-run, but bad in the short-run.

Changes needed to be made to have the code work for your system:
* Change motor output pins in the source code with your appropriate code.
* PID values tuned according to our system, so if you plan on using change the PID values accordingly to your system. 
