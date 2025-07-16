#include <ros/ros.h>
#include "g1_control/joint_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_controller");
    
    try
    {
        g1_control::JointController controller;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Joint Controller exception: %s", e.what());
        return 1;
    }
    
    ROS_INFO("Joint Controller shutting down.");
    return 0;
} 