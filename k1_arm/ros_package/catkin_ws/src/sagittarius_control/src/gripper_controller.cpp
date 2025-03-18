#include <ros/ros.h>
#include <std_srvs/Trigger.h>  // Add this include
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>

class GripperController {
public:
    GripperController(ros::NodeHandle& nh, const std::string& port) : 
        arm_(port, 1000000, 500, 5)
    {
        open_srv_ = nh.advertiseService("gripper/open", &GripperController::openCB, this);
        close_srv_ = nh.advertiseService("gripper/close", &GripperController::closeCB, this);
    }

private:
    bool openCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        arm_.arm_set_gripper_linear_position(0.0);
        res.success = true;
        res.message = "Gripper opened";
        return true;
    }

    bool closeCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        arm_.arm_set_gripper_linear_position(-0.068);
        res.success = true;
        res.message = "Gripper closed";
        return true;
    }

    sdk_sagittarius_arm::SagittariusArmReal arm_;
    ros::ServiceServer open_srv_, close_srv_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper_controller");
    ros::NodeHandle nh;
    
    std::string port;
    nh.param<std::string>("port", port, "/dev/ttyACM0");
    
    GripperController controller(nh, port);
    ros::spin();
    return 0;
}