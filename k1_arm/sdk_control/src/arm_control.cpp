#include "sdk_sagittarius_arm/sdk_sagittarius_arm_log.h"
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_real.h"
#include "sdk_sagittarius_arm/modern_robotics.h"
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <thread>

using namespace std;

bool running = true;
sdk_sagittarius_arm::SagittariusArmReal* arm_ptr = nullptr;

void signalHandler(int signum) {
    cout << "\nSignal (" << signum << ") received. Cleaning up...\n";
    running = false;
}

void holdPositionWhileGripper(sdk_sagittarius_arm::SagittariusArmReal& arm, float position[7], float gripper_pos, int duration_ms) {
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count() < duration_ms) {
        arm.SetAllServoRadian(position);
        arm.arm_set_gripper_linear_position(gripper_pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    signal(SIGINT, signalHandler);
    
    sdk_sagittarius_arm::SagittariusArmReal arm("/dev/ttyACM0", 1000000, 500, 5);
    arm_ptr = &arm;
    sdk_sagittarius_arm::SagittariusArmKinematics kinematics(0.06, 0, 0);

    // Define all positions
    float home_position[7] = {0, 0, -1.3, 0.02, 1.57, 0, 0};  // Working position
    float reset_position[7] = {0, 1.4, -1.47, 0.02, 1.57, 0, 0};  // Default/reset position
    float look_down[7] = {0, -0.8, -0.5, 0, 1.2, 0, 0};
    float reach_down[7] = {0, -1.4, 0.3, 0, 1.2, 0, 0};
    float current_position[7];
    memcpy(current_position, reset_position, sizeof(reset_position));

    cout << "Starting system...\n";
    
    // Move to reset position first
    cout << "Moving to reset position...\n";
    arm.SetAllServoRadian(reset_position);
    sleep(3);

    while(running) {
        char input;
        cout << "\nCommands available:\n";
        cout << "'s' - Start pick and place sequence\n";
        cout << "'r' - Return to reset position\n";
        cout << "'q' - Quit\n";
        cout << "Enter command: ";
        cin >> input;

        if (input == 'q') {
            break;
        }
        else if (input == 'r') {
            cout << "Returning to reset position...\n";
            arm.SetAllServoRadian(reset_position);
            memcpy(current_position, reset_position, sizeof(reset_position));
            sleep(3);
            continue;
        }
        else if (input == 's') {
            // First move to home/working position
            cout << "Moving to working position...\n";
            arm.SetAllServoRadian(home_position);
            sleep(3);

            // Open gripper while maintaining position
            cout << "Opening gripper...\n";
            holdPositionWhileGripper(arm, home_position, 0.0, 4000);  // 4 seconds for gripper operations

            // Look down
            cout << "Looking down at target...\n";
            arm.SetAllServoRadian(look_down);
            memcpy(current_position, look_down, sizeof(look_down));
            sleep(3);

            // Move down
            cout << "Moving down to target...\n";
            arm.SetAllServoRadian(reach_down);
            memcpy(current_position, reach_down, sizeof(reach_down));
            sleep(3);

            // Close gripper while maintaining position
            cout << "Closing gripper...\n";
            holdPositionWhileGripper(arm, reach_down, -0.068, 4000);  // 4 seconds for gripper operations
            
            // Extra delay to ensure gripper is fully closed
            sleep(2);

            // Move up
            cout << "Moving up...\n";
            arm.SetAllServoRadian(look_down);
            memcpy(current_position, look_down, sizeof(look_down));
            sleep(3);

            // Return to working position
            cout << "Returning to working position...\n";
            arm.SetAllServoRadian(home_position);
            memcpy(current_position, home_position, sizeof(home_position));
            sleep(3);

            cout << "Sequence completed. Holding position...\n";
        }
    }

    // Return to reset position before shutting down
    cout << "Returning to reset position before shutdown...\n";
    arm.SetAllServoRadian(reset_position);
    sleep(3);

    return 0;
}
