#include <PID.h>

// Simple PID library to create a flight controller using an accelerometer and gyrometer

// Constructor
PID::PID(double startPoint, double proportionalGain, double integralGain, double derivativeGain)
    :startPoint_(startPoint), proportionalGain_(proportionalGain), integralGain_(integralGain), derivativeGain_(derivativeGain)
{

}

// Default Constructor
PID::PID()
    :startPoint_(0)
{

}

// Set as bool to determine bad data
bool PID::computeOutput(double input, double* output){
    unsigned long currTime = millis();
    double elapsedTime = ((double) currTime - (double) prevTime_) / 1000;
    prevTime_ = currTime;

    // Prevent the usage of stale data
    if((float) elapsedTime <= sampleTiming_){
        return false;
    }else{
        // Calculate the error
        double currentErr = startPoint_ - input;
        integralErr_ += currentErr * elapsedTime;
        double derivativeErr = (currentErr - prevError_)/elapsedTime;
        prevError_ = currentErr;

        // Bounds checking
        // Check integral error
        // if(integralErr_ > 400){
        //     integralErr_ = 400;
        // }else if(integralErr_ < -400){
        //     integralErr_ = -400;
        // }

        // Sum for PID control
        *output = (proportionalGain_ * currentErr) + (integralGain_ * integralErr_) + (derivativeGain_ * derivativeErr);
        
        // Check output bounds
        if(*output > 200){
            *output = 200;
        }else if(*output < -200){
            *output = -200;
        }
        
        return true;
    }
}