#include "PIDController.h"

// Constructor with default parameters
PIDController::PIDController(double kp, double ki, double kd,
                             double sampleTime,
                             double outputMin, double outputMax,
                             double integralMin, double integralMax,
                             double derivativeFilterCoefficient,
                             double setpointWeightP,
                             double setpointWeightD)
    : kp_(kp), ki_(ki), kd_(kd),
      sampleTime_(sampleTime),
      outputMin_(outputMin), outputMax_(outputMax),
      integralMin_(integralMin), integralMax_(integralMax),
      derivativeFilterCoefficient_(derivativeFilterCoefficient),
      setpointWeightP_(setpointWeightP),
      setpointWeightD_(setpointWeightD),
      integral_(0.0),
      prevError_(0.0),
      prevMeasurement_(0.0),
      prevDerivative_(0.0),
      lastTime_(std::chrono::steady_clock::now())
{}

// Setters for PID coefficients
void PIDController::setKp(double kp) { kp_ = kp; }
void PIDController::setKi(double ki) { ki_ = ki; }
void PIDController::setKd(double kd) { kd_ = kd; }

// Setters for output limits
void PIDController::setOutputLimits(double min, double max) {
    outputMin_ = min;
    outputMax_ = max;
}

// Setters for integral windup limits
void PIDController::setIntegralLimits(double min, double max) {
    integralMin_ = min;
    integralMax_ = max;
}

// Set sample time in seconds
void PIDController::setSampleTime(double sampleTime) {
    sampleTime_ = sampleTime;
}

// Set derivative filter coefficient (0 for no filter)
void PIDController::setDerivativeFilterCoefficient(double alpha) {
    derivativeFilterCoefficient_ = alpha;
}

// Set setpoint weighting
void PIDController::setSetpointWeights(double weightP, double weightD) {
    setpointWeightP_ = weightP;
    setpointWeightD_ = weightD;
}

// Reset the controller state
void PIDController::reset() {
    integral_ = 0.0;
    prevError_ = 0.0;
    prevMeasurement_ = 0.0;
    prevDerivative_ = 0.0;
    lastTime_ = std::chrono::steady_clock::now();
}

/**
 * Compute the PID output.
 * @param setpoint The desired setpoint.
 * @param measurement The current measurement.
 * @return The PID controller output.
 */
double PIDController::compute(double setpoint, double measurement) {
    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = currentTime - lastTime_;
    double dt = elapsed.count();

    if (dt >= sampleTime_) {
        double error = measurement - setpoint; // Positive when measurement > setpoint

        // Update integral with anti-windup
        integral_ += ki_ * error * dt;
        integral_ = clamp(integral_, integralMin_, integralMax_);

        // Calculate derivative (derivative of error)
        double derivative = 0.0;
        if (dt > 0.0) {
            derivative = (error - prevError_) / dt;
            // Apply derivative filtering (e.g., low-pass filter)
            derivative = derivativeFilterCoefficient_ * derivative + (1.0 - derivativeFilterCoefficient_) * prevDerivative_;
        }

        // Compute PID output
        double output = kp_ * error + integral_ + kd_ * derivative;

        // Clamp output to limits
        output = clamp(output, outputMin_, outputMax_);

        // Save state for next iteration
        prevError_ = error;
        prevDerivative_ = derivative;
        lastTime_ = currentTime;

        return output;
    } else {
        // Not enough time has passed, return previous output or zero
        return 0.0;
    }
}

// Utility clamp function
double PIDController::clamp(double value, double min, double max) {
    return std::max(min, std::min(value, max));
}
