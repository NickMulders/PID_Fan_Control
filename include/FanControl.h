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
#define FAN_GPIO_PIN 1          // WiringPi pin 1 (BCM_GPIO 18)

// Temperature Thresholds (in degrees Celsius)
#define TEMP_THRESHOLD 30       // Temperature to start fan

// PWM Parameters
#define PWM_RANGE 100           // PWM range (0-100)

// Sampling Interval
#define SAMPLE_INTERVAL_MS 2000  // 2 seconds

/* ========== Global Variables ========== */
extern volatile sig_atomic_t LOG_ENABLED;

/* ========== Function Prototypes ========== */

class FanControl {
public:
    FanControl();
    ~FanControl();
    void Run();

private:
    double ReadCPUTemperature();
    void InitializeGPIO();
    int ComputeFanPWM(double temperature);
    static void ToggleLogging(int signum);

    // PID Controller instance
    PIDController pid_;
};

#endif // FAN_CONTROL_H
