#ifndef FANCONTROL_H
#define FANCONTROL_H

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

/* ========== Definitions ========== */

// GPIO Pin Definitions
#define FAN_GPIO_PIN 1          // WiringPi pin 1 (BCM_GPIO 18)

// Temperature Thresholds (in degrees Celsius)
#define TEMP_THRESHOLD 30       // Temperature to start fan

// PD Controller Parameters
//TODO: Tune the PD values (Maybe add Integral if necessary)
#define KP 2.0                  // Proportional gain
#define KD 0.5                  // Derivative gain

// PWM Parameters
#define PWM_RANGE 100           // PWM range (0-100)
#define PWM_FREQUENCY 100       // PWM frequency in Hz

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
    int MapTemperatureToPWM(double temperature, double& previousError);
    static void ToggleLogging(int signum);
};

#endif // FANCONTROL_H
