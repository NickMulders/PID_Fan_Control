#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H

#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <cstring>
#include <csignal>
#include "PIDController.h" // Include the PIDController

/* ========== Definitions ========== */

// GPIO Pin Definitions
#define FAN_GPIO_PIN 26             // WiringPi pin 26 (BCM_GPIO 12)

// Temperature Thresholds (in degrees Celsius)
#define TEMP_THRESHOLD 30          // Temperature threshold in °C to turn on/off the fan

// PWM Parameters
#define PWM_START_VALUE      20    // Starting PWM value (%)
#define PWM_THRESHOLD        50    // Threshold to apply PWM kick (%)
#define KICK_PWM             60    // PWM value for the kick (%)
#define KICK_DURATION_MS     100   // Duration of the kick in milliseconds
#define MAX_PWM              100   // Maximum PWM value (%)
#define MIN_PWM              PWM_START_VALUE // Minimum PWM value (%)

// Sampling Interval
#define SAMPLE_INTERVAL_MS  1000    // 1 second

/* ========== Global Variables ========== */
extern volatile sig_atomic_t LOG_ENABLED;

/* ========== Function Prototypes ========== */

class FanControl {
public:
    FanControl();
    ~FanControl();
    void Run();

private:
    int lastPwmValue = 0;
    bool isFanRunning = false;

    double ReadCPUTemperature();
    void InitializeGPIO();
    int ComputeFanPWM(double temperature);
    static void ToggleLogging(int signum);

    // PID Controller instance
    PIDController pid_;
};

#endif // FAN_CONTROL_H
