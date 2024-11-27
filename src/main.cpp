#include "FanControl.h"

/* ========== Main ========== */

/**
 * @brief Entry point of the program.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char* argv[]) {
    //TODO: remove the UNUSED parameters warning

    // Create FanControl object and run
    FanControl fanControl;
    fanControl.Run();

    return 0;
}
