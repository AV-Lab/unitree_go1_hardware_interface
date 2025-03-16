#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>  
#include <std_msgs/Float64MultiArray.h> 
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>

class ArmController {
public:
    ArmController(ros::NodeHandle& nh, const std::string& port) : 
        arm_(port, 1000000, 500, 5)
    {
        // Individual joint subscribers
        joints_.resize(6);
        subs_.resize(6);
        
        subs_[0] = nh.subscribe("arm_control/base", 1, &ArmController::baseCB, this);
        subs_[1] = nh.subscribe("arm_control/shoulder", 1, &ArmController::shoulderCB, this);
        subs_[2] = nh.subscribe("arm_control/elbow", 1, &ArmController::elbowCB, this);
        subs_[3] = nh.subscribe("arm_control/wrist_roll", 1, &ArmController::wristRollCB, this);
        subs_[4] = nh.subscribe("arm_control/wrist_pitch", 1, &ArmController::wristPitchCB, this);
        subs_[5] = nh.subscribe("arm_control/wrist_yaw", 1, &ArmController::wristYawCB, this);


        // Array-based subscriber 
        array_sub_ = nh.subscribe("arm_control/joint_angles", 1, &ArmController::jointArrayCB, this);

        // Reset service
        reset_srv_ = nh.advertiseService("arm_control/reset", &ArmController::resetCB, this);
    }

private:
    void updateJoints() {
        float angles[6] = {
            joints_[0], joints_[1], joints_[2],
            joints_[3], joints_[4], joints_[5]
        };
        arm_.SetAllServoRadian(angles);
    }

    void jointArrayCB(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if(msg->data.size() != 6) {
            ROS_WARN("Invalid joint angles message - need 6 values");
            return;
        }
        
        for(int i=0; i<6; i++) {
            joints_[i] = msg->data[i];
        }
        updateJoints();
    }

    // Joint callbacks
    void baseCB(const std_msgs::Float64::ConstPtr& msg) { joints_[0] = msg->data; updateJoints(); }
    void shoulderCB(const std_msgs::Float64::ConstPtr& msg) { joints_[1] = msg->data; updateJoints(); }
    void elbowCB(const std_msgs::Float64::ConstPtr& msg) { joints_[2] = msg->data; updateJoints(); }
    void wristRollCB(const std_msgs::Float64::ConstPtr& msg) { joints_[3] = msg->data; updateJoints(); }
    void wristPitchCB(const std_msgs::Float64::ConstPtr& msg) { joints_[4] = msg->data; updateJoints(); }
    void wristYawCB(const std_msgs::Float64::ConstPtr& msg) { joints_[5] = msg->data; updateJoints(); }

    // Reset service
    bool resetCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        float reset_angles[6] = {0.0, 1.4, -1.4, 0.0, 1.5, 0.0};  // Remove const
        arm_.SetAllServoRadian(reset_angles);
        res.success = true;
        res.message = "Arm reset to home position";
        return true;
    }

    sdk_sagittarius_arm::SagittariusArmReal arm_;
    std::vector<float> joints_;
    std::vector<ros::Subscriber> subs_;
    ros::Subscriber array_sub_;
    ros::ServiceServer reset_srv_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    
    std::string port;
    nh.param<std::string>("port", port, "/dev/ttyACM0");
    
    ArmController controller(nh, port);
    ros::spin();
    return 0;
}