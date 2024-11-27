#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <chrono>
#include <algorithm>

class PIDController {
public:
    // Constructor with default parameters
    PIDController(double kp = 2.0, double ki = 0.0, double kd = 0.5,    //TODO: Tune PID values
                 double sampleTime = 0.01, // in seconds
                 double outputMin = 0.0, double outputMax = 100.0,
                 double integralMin = -100.0, double integralMax = 100.0,
                 double derivativeFilterCoefficient = 0.0, // 0 for no filter
                 double setpointWeightP = 1.0,
                 double setpointWeightD = 1.0);

    // Setters for PID coefficients
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);

    // Setters for output limits
    void setOutputLimits(double min, double max);

    // Setters for integral windup limits
    void setIntegralLimits(double min, double max);

    // Set sample time in seconds
    void setSampleTime(double sampleTime);

    // Set derivative filter coefficient (0 for no filter)
    void setDerivativeFilterCoefficient(double alpha);

    // Set setpoint weighting
    void setSetpointWeights(double weightP, double weightD);

    // Reset the controller state
    void reset();

    // Compute the PID output.
    double compute(double setpoint, double measurement);

private:
    // PID coefficients
    double kp_;
    double ki_;
    double kd_;

    // Sample time in seconds
    double sampleTime_;

    // Output limits
    double outputMin_;
    double outputMax_;

    // Integral windup limits
    double integralMin_;
    double integralMax_;

    // Derivative filter coefficient (0 for no filter)
    double derivativeFilterCoefficient_;

    // Setpoint weighting
    double setpointWeightP_;
    double setpointWeightD_;

    // State variables
    double integral_;
    double prevError_;
    double prevMeasurement_;
    double prevDerivative_;
    std::chrono::steady_clock::time_point lastTime_;

    // Utility clamp function
    double clamp(double value, double min, double max);
};

#endif // PID_CONTROLLER_H
