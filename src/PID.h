#include <Arduino.h>

class PID{
    public:
    PID();
    PID(double startPoint, double proportionalGain, double integralGain, double derivativeGain);
    bool computeOutput(double input, double* output);
    unsigned long prevTime_;

    private:
    double startPoint_;
    float sampleTiming_ = 0.0005;
    double proportionalGain_;
    double integralGain_;
    double derivativeGain_;
    double integralErr_ = 0;
    double prevError_ = 0;
};