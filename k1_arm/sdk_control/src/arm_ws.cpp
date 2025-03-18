#include "sdk_sagittarius_arm/sdk_sagittarius_arm_log.h"
#include "sdk_sagittarius_arm/sdk_sagittarius_arm_real.h"
#include "uWebSockets/src/App.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <array>
#include <signal.h>

using json = nlohmann::json;
using namespace std;

// Define an empty user data type for uWS
struct PerSocketData {};

// Global variables
bool running = true;
sdk_sagittarius_arm::SagittariusArmReal* arm_ptr = nullptr;
float current_position[6];

// Joint limits and presets (same as before)
const float joint_min[6] = {-1.9, -1.5, -1.4, -2.88, -1.73, -3.1};
const float joint_max[6] = { 1.9,  1.3,  1.7,  2.82,  1.57,  3.1};

const float RESET_POSITION[6]    = {0.0, 0.0, -1.3, 0.0, 1.5, 0.0};
const float RESET_POSITION_R[6]  = {0.0, 1.4, -1.4, 0.0, 1.5, 0.0};
const float PRE_PICK_POSITION_1[6] = {0.0, 1.4, -1.3, 0.0, 1.5, 0.0};
const float PRE_PICK_POSITION[6] = {0.24, 1.4, 0.55, 0.0, -1.13, 0.0};
const float PICK_POSITION[6]     = {0.24, 0.23, 0.55, 0.0, -1.13, 0.0};


const float GRIPPER_OPEN   = 0.0;
const float GRIPPER_CLOSED = -0.068;

// Recorded point structure
struct RecordedPoint {
    std::array<float,6> joints;
    float gripper;
};

std::vector<RecordedPoint> recorded_path;
float current_gripper = GRIPPER_CLOSED;
bool auto_pick = true;
bool place_reset = true;

// Helper function to send joint positions to the arm
void updateArmJoints(const float joints[6]) {
    // Cast away const to match the expected parameter type.
    arm_ptr->SetAllServoRadian(const_cast<float*>(joints));
    memcpy(current_position, joints, sizeof(current_position));
}

// CHANGED: Modified signal handler to reset the arm before exiting.
void signalHandler(int signum) {
    cout << "\nSignal (" << signum << ") received. Resetting arm before exit...\n";
    updateArmJoints(RESET_POSITION_R);
    arm_ptr->arm_set_gripper_linear_position(GRIPPER_CLOSED);
    sleep(2);  // Allow time for the arm to move.
    exit(0);
}


// Process incoming JSON commands from the WebSocket
void processCommand(const json& cmd) {
    try {
        string action = cmd.at("action").get<string>();
        if (action == "set_joints") {
            auto joints = cmd.at("joints").get<std::vector<float>>();
            if (joints.size() != 6) {
                cerr << "Error: Expected 6 joint values" << endl;
                return;
            }
            updateArmJoints(joints.data());
        }
        else if (action == "open_gripper") {
            current_gripper = GRIPPER_OPEN;
            arm_ptr->arm_set_gripper_linear_position(current_gripper);
        }
        else if (action == "close_gripper") {
            current_gripper = GRIPPER_CLOSED;
            arm_ptr->arm_set_gripper_linear_position(current_gripper);
        }
        else if (action == "reset") {
            updateArmJoints(RESET_POSITION_R);
            sleep(2);
        }
        else if (action == "pick_place") {
            if (auto_pick) {
                cout << "\n--- Starting Automatic PICK Sequence ---" << endl;
                cout << "\nOpening gripper to pick up item..." << endl;
                current_gripper = GRIPPER_OPEN;
                arm_ptr->arm_set_gripper_linear_position(current_gripper);
                updateArmJoints(RESET_POSITION_R);
                sleep(2);
                updateArmJoints(PRE_PICK_POSITION);
                sleep(2);
		        updateArmJoints(PICK_POSITION);
                sleep(3);               
                auto_pick = false;
            } else if (place_reset){
                updateArmJoints(PRE_PICK_POSITION);
                sleep(2);
                updateArmJoints(RESET_POSITION_R);
                sleep(2);
                place_reset = false;
                cout << "\nPick sequence completed." << endl;
            }
            else {
                cout << "\n--- Starting Automatic DROP Sequence ---" << endl;
             
                updateArmJoints(PRE_PICK_POSITION);
                sleep(2);
                updateArmJoints(PICK_POSITION);
                sleep(2);
                cout << "\nOpening gripper to drop item..." << endl;
                current_gripper = GRIPPER_OPEN;
                arm_ptr->arm_set_gripper_linear_position(current_gripper);
                sleep(3);
                updateArmJoints(PRE_PICK_POSITION);
                sleep(2);
                updateArmJoints(RESET_POSITION_R);
                sleep(2);
                auto_pick = true;
                place_reset = true;
                cout << "\nDrop sequence completed." << endl;
            }
        }
        else if (action == "record") {
            RecordedPoint point;
            for (int i = 0; i < 6; i++) {
                point.joints[i] = current_position[i];
            }
            point.gripper = current_gripper;
            recorded_path.push_back(point);
            cout << "\nRecorded current position. Total recorded points: " << recorded_path.size() << endl;
        }
        else if (action == "playback") {
            if (recorded_path.empty()) {
                cout << "\nNo recorded path to play." << endl;
                return;
            }
            cout << "\nPlaying back recorded path..." << endl;
            for (auto &pt : recorded_path) {
                arm_ptr->SetAllServoRadian(pt.joints.data());
                arm_ptr->arm_set_gripper_linear_position(pt.gripper);
                sleep(3);
            }
            cout << "\nPlayback complete." << endl;
        }
        else if (action == "clear_recording") {
            recorded_path.clear();
            cout << "\nCleared recorded path." << endl;
        }
        else {
            cout << "\nUnknown action received: " << action << endl;
        }
    } catch (std::exception& e) {
        cerr << "Exception in processCommand: " << e.what() << endl;
    }
}

int main() {
    signal(SIGINT, signalHandler);
    
    // Initialize the arm.
    sdk_sagittarius_arm::SagittariusArmReal arm("/dev/ttyACM0", 1000000, 500, 5);
    arm_ptr = &arm;

    // Start from current  position.
    memcpy(current_position, RESET_POSITION_R, sizeof(RESET_POSITION_R));
    updateArmJoints(current_position);
    arm.arm_set_gripper_linear_position(current_gripper);
    sleep(1);

    // Start the WebSocket server on port 9002.
    uWS::App().ws<PerSocketData>("/*", {
        .open = [](auto* ws) {
            cout << "Client connected" << endl;
        },
        .message = [](auto* ws, std::string_view message, uWS::OpCode opCode) {
            try {
                auto cmd = json::parse(message);
                processCommand(cmd);
            } catch (std::exception &e) {
                cerr << "Error parsing JSON: " << e.what() << endl;
            }
        },
        .close = [](auto* ws, int code, std::string_view message) {
            cout << "Client disconnected" << endl;
        }
    }).listen(9002, [](auto* token) {
        if (token) {
            cout << "WebSocket server listening on port 9002" << endl;
        } else {
            cerr << "Failed to listen on port 9002" << endl;
        }
    }).run();

    // On exit, reset the arm.
    cout << "\nResetting arm to default position..." << endl;
    updateArmJoints(RESET_POSITION_R);
    sleep(2);
    cout << "Exiting control. Goodbye!" << endl;
    return 0;
}
