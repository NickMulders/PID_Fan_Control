#include "FanControl.h"

/* ========== Functionality ========== */

/**
 * @brief Constructor. Initializes GPIO, PID controller.
 */
FanControl::FanControl()
    : pid_(5.30562491183526, 5.21069262830874, 1.72236927768401, 0.5, 0.0, MAX_PWM, -50.0, 50.0, 0.1, 1.0, 1.0) {
    InitializeGPIO();

    // Configure PID Controller
    pid_.setOutputLimits(0.0, MAX_PWM);           // PWM output between 0 and 100
    pid_.setIntegralLimits(-50.0, 50.0);         // Integral windup limits
    pid_.setSampleTime(0.02);                    // 20 ms sample time based on plant dynamics
    pid_.setDerivativeFilterCoefficient(0.1);    // Derivative filter coefficient
    pid_.setSetpointWeights(1.0, 1.0);           // Setpoint weighting
}

/**
 * @brief Destructor. Cleans up GPIO settings.
 */
FanControl::~FanControl() {
    // Cleanup: Turn off the fan
    softPwmWrite(FAN_GPIO_PIN, 0);
    std::cout << "Fans have been disabled." << std::endl;
}

/**
 * @brief Runs the main control loop.
 */
void FanControl::Run() {
    // Disable buffering for std::cout to ensure real-time log output
    std::cout << std::unitbuf;

    std::cout << "Starting PID GPIO Fan Control Application..." << std::endl;

    // Static variable to track if PWM Kick has been applied
    static bool hasKicked = false;

    while (true) {
        double currentTemp = ReadCPUTemperature();
        int pwmValue = ComputeFanPWM(currentTemp);

        // Check if the fan needs to be started
        if (pwmValue > 0) {
            if (!isFanRunning) {
                std::cout << "Fans have been enabled." << std::endl;
                isFanRunning = true;
                hasKicked = false;  // Reset the kick flag when fans are enabled
            }

            if (pwmValue != lastPwmValue) {
                // Check if the fan needs to receive a KICK start
                if (pwmValue < PWM_THRESHOLD && !hasKicked) {
                    softPwmWrite(FAN_GPIO_PIN, KICK_PWM);
                    std::cout << "Applying KICK PWM: " << KICK_PWM << "%" << std::endl;

                    // Wait for the kick duration to allow the fan to start
                    std::this_thread::sleep_for(std::chrono::milliseconds(KICK_DURATION_MS));

                    hasKicked = true;  // Mark that the kick has been applied
                }

                // Set the desired PWM value to control the fan speed
                lastPwmValue = pwmValue;
                softPwmWrite(FAN_GPIO_PIN, pwmValue);
            }

            std::cout << "CPU Temp: " << currentTemp << "°C | Fan PWM: " << pwmValue << "%" << std::endl;
        } else {
            if (isFanRunning) {
                softPwmWrite(FAN_GPIO_PIN, 0);
                isFanRunning = false;
                hasKicked = false;  // Reset the kick flag when fans are disabled
                std::cout << "Fans have been disabled." << std::endl;
            }
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

    std::cout << "GPIO initialized successfully on pin " << FAN_GPIO_PIN << "." << std::endl;
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
