#include <PIDScratch.h>

// Simple PID library to create a flight controller using an accelerometer and gyrometer

class PID{
    public:
    PID();
    PID(double* startPoint);
    computeOutput(double input);

    private:
    double* startPoint_;
    double* output_;
    double prevTime_;
    double sampleTiming_;
    double proportionalGain_;
    double integralGain_;
    double derivativeGain_;
}

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
bool PID::computeOutput(double input){
    double currTime = millis();
    elapsedTime_ = currTime - prevTime_;
    prevTime_ = currTime;

    // Prevent the usage of stale data
    if(elapsedTime > sampleTiming_){
        return output_;
    }else{
        // Calculate the error
        double err = startPoint_ - input;

    }

        
}