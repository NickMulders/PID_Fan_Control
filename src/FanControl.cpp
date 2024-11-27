#include "FanControl.h"

/* ========== Global Variables ========== */
volatile sig_atomic_t LOG_ENABLED = 0; // Logging is disabled by default

/* ========== Functionality ========== */

/**
 * @brief Constructor. Initializes GPIO, PID controller, and sets up signal handling.
 */
FanControl::FanControl()
    : pid_(2.0, 0.5, 1.0, 0.5, 0.0, PWM_RANGE, -50.0, 50.0, 0.1, 1.0, 1.0) {
    // Set up signal handler for toggling logging (e.g., SIGHUP)
    std::signal(SIGHUP, FanControl::ToggleLogging);
    InitializeGPIO();

    // Configure PID Controller
    pid_.setOutputLimits(0.0, PWM_RANGE);          // PWM output between 0 and 100
    pid_.setIntegralLimits(-50.0, 50.0);          // Integral windup limits
    pid_.setSampleTime(0.5);                      // 0.5 seconds
    pid_.setDerivativeFilterCoefficient(0.1);      // Derivative filter coefficient
    pid_.setSetpointWeights(1.0, 1.0);            // Setpoint weighting
}

/**
 * @brief Destructor. Cleans up GPIO settings.
 */
FanControl::~FanControl() {
    // Cleanup: Turn off the fan
    softPwmWrite(FAN_GPIO_PIN, 0); // Ensure fan is turned off
}

/**
 * @brief Runs the main control loop.
 */
void FanControl::Run() {
    while (true) {
        double currentTemp = ReadCPUTemperature();
        int pwmValue = ComputeFanPWM(currentTemp);

        softPwmWrite(FAN_GPIO_PIN, pwmValue);

        if (LOG_ENABLED) {
            std::cout << "CPU Temp: " << currentTemp << "°C | Fan PWM: " << pwmValue << "%" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(SAMPLE_INTERVAL_MS));
    }
}

/**
 * @brief Reads the current CPU temperature.
 * @return Current temperature in degrees Celsius.
 */
double FanControl::ReadCPUTemperature() {
    std::ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
    std::string tempStr;
    double temp = 0.0;

    if (tempFile.is_open()) {
        std::getline(tempFile, tempStr);
        try {
            temp = std::stod(tempStr) / 1000.0; // Convert millidegrees to degrees
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid temperature format." << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Temperature value out of range." << std::endl;
        }
        tempFile.close();
    } else {
        std::cerr << "Unable to read CPU temperature." << std::endl;
    }

    return temp;
}

/**
 * @brief Initializes GPIO settings for the fan control.
 */
void FanControl::InitializeGPIO() {
    if (wiringPiSetup() == -1) {
        std::cerr << "WiringPi initialization failed!" << std::endl;
        exit(1);
    }

    // Setup the GPIO pin for PWM output
    pinMode(FAN_GPIO_PIN, PWM_OUTPUT);
    if (softPwmCreate(FAN_GPIO_PIN, 0, PWM_RANGE) != 0) {
        std::cerr << "Failed to initialize soft PWM on pin " << FAN_GPIO_PIN << std::endl;
        exit(1);
    }
}

/**
 * @brief Computes the PWM value for the fan using the PID controller.
 * @param temperature Current temperature in degrees Celsius.
 * @return PWM value corresponding to the temperature.
 */
int FanControl::ComputeFanPWM(double temperature) {
    if (temperature < TEMP_THRESHOLD) {
        // Turn off GPIO fan
        pid_.reset(); // Reset PID controller to prevent integral windup
        return 0;
    } else {
        double setpoint = TEMP_THRESHOLD;
        double pwmOutput = pid_.compute(setpoint, temperature);

        // Clamp PWM output to PWM_RANGE
        pwmOutput = std::max(0.0, std::min(pwmOutput, static_cast<double>(PWM_RANGE)));

        return static_cast<int>(pwmOutput);
    }
}

/**
 * @brief Toggles the logging state.
 * @param signum Signal number.
 */
void FanControl::ToggleLogging(int signum) {
    if (signum == SIGHUP) {
        LOG_ENABLED = !LOG_ENABLED;
        std::cout << "Logging " << (LOG_ENABLED ? "enabled." : "disabled.") << std::endl;
    }
}
