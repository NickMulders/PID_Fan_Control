#include "FanControl.h"

/* ========== Global Variables ========== */
volatile sig_atomic_t LOG_ENABLED = false; // Logging is disabled by default

/* ========== Functionality ========== */

/**
 * @brief Constructor. Initializes GPIO, PID controller, and sets up signal handling.
 */
FanControl::FanControl()
    : pid_(2.0, 0.5, 1.0, 0.5, 0.0, MAX_PWM, -50.0, 50.0, 0.1, 1.0, 1.0) {
    // Set up signal handler for toggling logging (e.g., SIGHUP)
    std::signal(SIGHUP, FanControl::ToggleLogging);
    InitializeGPIO();

    // Configure PID Controller
    pid_.setOutputLimits(0.0, MAX_PWM);          // PWM output between 0 and 100
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
    softPwmWrite(FAN_GPIO_PIN, 0);
}

/**
 * @brief Runs the main control loop.
 */
void FanControl::Run() {
    while (true) {
        double currentTemp = ReadCPUTemperature();
        int pwmValue = ComputeFanPWM(currentTemp);

        // Check if the fan needs to be started
        if (pwmValue > 0) {
            if (pwmValue != lastPwmValue) {
                // Check if the fan needs to receive a KICK start
                if (pwmValue < PWM_THRESHOLD) {
                    softPwmWrite(FAN_GPIO_PIN, KICK_PWM);
                    if (LOG_ENABLED) {
                        std::cout << "Applying KICK PWM: " << KICK_PWM << "%" << std::endl;
                    }

                    // Wait for the kick duration to allow the fan to start
                    std::this_thread::sleep_for(std::chrono::milliseconds(KICK_DURATION_MS));
                }

                // Set the desired PWM value to control the fan speed
                lastPwmValue = pwmValue;
                softPwmWrite(FAN_GPIO_PIN, pwmValue);
                isFanRunning = true;
            }
        } else {
            if (isFanRunning) {
                softPwmWrite(FAN_GPIO_PIN, 0);
                isFanRunning = false;
            }
        }
        
        if (LOG_ENABLED) {
            std::cout << "CPU Temp: " << currentTemp << "°C | Fan PWM: " << pwmValue << "%" << std::endl;
        }

        // Calculate the remaining sleep time to maintain consistent loop duration
        size_t sleepDuration = (pwmValue < PWM_THRESHOLD) ? 
                             (SAMPLE_INTERVAL_MS - KICK_DURATION_MS) : 
                             SAMPLE_INTERVAL_MS;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepDuration));
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
    if (softPwmCreate(FAN_GPIO_PIN, 0, MAX_PWM) != 0) {
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

        // Clamp PWM output to MAX_PWM
        pwmOutput = std::max(0.0, std::min(pwmOutput, static_cast<double>(MAX_PWM)));

        // Ensure PWM is at least PWM_START_VALUE
        pwmOutput = std::max(static_cast<double>(PWM_START_VALUE), pwmOutput);

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
