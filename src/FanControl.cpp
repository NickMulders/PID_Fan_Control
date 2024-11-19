#include "FanControl.h"

/* ========== Global Variables ========== */
volatile sig_atomic_t LOG_ENABLED = 0; // Quiet by default


/* ========== Functionality ========== */

/**
 * @brief Constructor. Initializes GPIO and sets up signal handling.
 */
FanControl::FanControl() {
    // Set up signal handler for toggling logging (e.g., SIGHUP)
    // This allows the program to handle SIGHUP signals asynchronously.
    std::signal(SIGHUP, FanControl::ToggleLogging);
    InitializeGPIO();
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
    double previousError = 0.0;

    while (true) {
        double currentTemp = ReadCPUTemperature();
        int pwmValue = MapTemperatureToPWM(currentTemp, previousError); // Derivative is handled inside

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
        temp = std::stod(tempStr) / 1000.0; // Convert millidegrees to degrees
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
    softPwmCreate(FAN_GPIO_PIN, 0, PWM_RANGE);
}

/**
 * @brief Maps the current temperature to a PWM value using a PD controller.
 * @param temperature Current temperature in degrees Celsius.
 * @param previousError Previous temperature error.
 * @return PWM value corresponding to the temperature.
 */
int FanControl::MapTemperatureToPWM(double temperature, double& previousError) {
    double error = 0.0;
    double dError = 0.0;

    if (temperature < TEMP_THRESHOLD) {
        // Temperature below threshold; turn off fan
        previousError = 0.0;
        return 0;
    } else {
        // Calculate error (how much we are above the threshold)
        error = temperature - TEMP_THRESHOLD;

        // Calculate derivative (rate of change)
        dError = error - previousError;

        // PD Control
        double output = (KP * error) + (KD * dError);

        // Clamp output to PWM range
        if (output > PWM_RANGE) output = PWM_RANGE;
        if (output < 0) output = 0;

        previousError = error;

        return static_cast<int>(output);
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


/* ========== Main ========== */

/**
 * @brief Entry point of the program.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char* argv[]) {
    // Create FanControl object and run
    FanControl fanControl;
    fanControl.Run();

    return 0;
}
