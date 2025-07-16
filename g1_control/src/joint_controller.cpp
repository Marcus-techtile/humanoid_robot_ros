#include "g1_control/joint_controller.h"

namespace g1_control
{

JointController::JointController() : private_nh_("~")
{
    // Load parameters from ROS parameter server
    loadParameters();
    
    // Initialize robot joint names (same order as in Python controllers)
    initializeJointNames();
    
    // Initialize publishers and subscribers based on configuration
    if (joint_command_topic_.find("joint_states") != std::string::npos || 
        joint_command_topic_.find("JointState") != std::string::npos)
    {
        // Subscribe to JointState messages (e.g., from joint_state_publisher_gui)
        joint_command_sub_ = nh_.subscribe<sensor_msgs::JointState>(
            joint_command_topic_, 10, &JointController::jointCommandCallback, this);
        ROS_INFO("Subscribed to JointState topic: %s", joint_command_topic_.c_str());
    }
    else
    {
        // Subscribe to Float64MultiArray messages (direct joint commands)
        joint_command_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>(
            joint_command_topic_, 10, &JointController::directCommandCallback, this);
        ROS_INFO("Subscribed to Float64MultiArray topic: %s", joint_command_topic_.c_str());
    }
    
    joint_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(joint_output_topic_, 1);
    
    // Initialize control timer only if not in direct mode
    if (!direct_mode_)
    {
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                       &JointController::controlCallback, this);
        ROS_INFO("Control loop enabled at %.1f Hz", control_rate_);
    }
    else
    {
        ROS_INFO("Direct mode enabled - commands forwarded immediately");
    }
    
    // Initialize joint positions map
    for (const auto& joint_name : robot_joint_names_)
    {
        command_joint_positions_[joint_name] = 0.0;
    }
    
    ROS_INFO("Joint Controller initialized");
    ROS_INFO("Command topic: %s", joint_command_topic_.c_str());
    ROS_INFO("Output topic: %s", joint_output_topic_.c_str());
    ROS_INFO("Controlling %zu joints", robot_joint_names_.size());
}

void JointController::loadParameters()
{
    // Control parameters
    private_nh_.param("control_rate", control_rate_, 50.0);  // Hz
    private_nh_.param("max_joint_velocity", max_joint_velocity_, 1.0);  // rad/s
    private_nh_.param("position_tolerance", position_tolerance_, 0.001);  // rad
    
    // Topic configuration
    private_nh_.param<std::string>("joint_command_topic", joint_command_topic_, "/joint_states");
    private_nh_.param<std::string>("joint_output_topic", joint_output_topic_, "/joint_commands");
    
    // Control mode
    private_nh_.param("direct_mode", direct_mode_, false);
    
    ROS_INFO("Loaded parameters:");
    ROS_INFO("  - Control rate: %.1f Hz", control_rate_);
    ROS_INFO("  - Max joint velocity: %.3f rad/s", max_joint_velocity_);
    ROS_INFO("  - Position tolerance: %.6f rad", position_tolerance_);
    ROS_INFO("  - Direct mode: %s", direct_mode_ ? "true" : "false");
}

void JointController::initializeJointNames()
{
    // Check if custom joint names are provided
    std::vector<std::string> custom_joint_names;
    if (private_nh_.getParam("robot_joint_names", custom_joint_names) && !custom_joint_names.empty())
    {
        robot_joint_names_ = custom_joint_names;
        ROS_INFO("Using custom joint names from parameter server (%zu joints)", robot_joint_names_.size());
    }
    else
    {
        // Default G1 humanoid robot joint names
        robot_joint_names_ = {
            // Left leg (6 DOF)
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            // Right leg (6 DOF)
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            // Waist (3 DOF)
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            // Left arm (7 DOF)
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            // Right arm (7 DOF)
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        };
        ROS_INFO("Using default G1 joint names (%zu joints)", robot_joint_names_.size());
    }
}

void JointController::jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Update joint positions from command source (e.g., joint_state_publisher_gui)
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        const std::string& joint_name = msg->name[i];
        
        // Check if this joint is in our robot's joint list
        if (command_joint_positions_.find(joint_name) != command_joint_positions_.end())
        {
            double command = msg->position[i];
            
            // Validate command for safety
            if (validateJointCommand(joint_name, command))
            {
                command_joint_positions_[joint_name] = command;
            }
        }
    }
    
    ROS_DEBUG_THROTTLE(1.0, "Received joint states for %zu joints", msg->name.size());
    
    // If in direct mode, immediately publish commands
    if (direct_mode_)
    {
        controlCallback(ros::TimerEvent());
    }
}

void JointController::directCommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Handle direct joint command array
    if (msg->data.size() != robot_joint_names_.size())
    {
        ROS_WARN("Received %zu commands but expected %zu", msg->data.size(), robot_joint_names_.size());
        return;
    }
    
    // Update joint positions from direct commands
    for (size_t i = 0; i < robot_joint_names_.size(); ++i)
    {
        const std::string& joint_name = robot_joint_names_[i];
        double command = msg->data[i];
        
        // Validate command for safety
        if (validateJointCommand(joint_name, command))
        {
            command_joint_positions_[joint_name] = command;
        }
    }
    
    ROS_DEBUG_THROTTLE(1.0, "Received direct commands for %zu joints", msg->data.size());
    
    // If in direct mode, immediately publish commands
    if (direct_mode_)
    {
        controlCallback(ros::TimerEvent());
    }
}

void JointController::controlCallback(const ros::TimerEvent& event)
{
    // Create joint command message
    std_msgs::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(robot_joint_names_.size());
    
    // Fill joint commands in the correct order
    for (size_t i = 0; i < robot_joint_names_.size(); ++i)
    {
        const std::string& joint_name = robot_joint_names_[i];
        
        // Get the position from command input, default to 0.0 if not available
        auto it = command_joint_positions_.find(joint_name);
        if (it != command_joint_positions_.end())
        {
            cmd_msg.data[i] = it->second;
        }
        else
        {
            cmd_msg.data[i] = 0.0;
        }
    }
    
    // Publish joint commands
    joint_cmd_pub_.publish(cmd_msg);
    
    // Log status periodically (only if not in direct mode to avoid spam)
    if (!direct_mode_)
    {
        static ros::Time last_log_time = ros::Time::now();
        if ((ros::Time::now() - last_log_time).toSec() > 5.0)
        {
            ROS_INFO("Joint Controller active - controlling %zu joints", robot_joint_names_.size());
            last_log_time = ros::Time::now();
        }
    }
}

bool JointController::validateJointCommand(const std::string& joint_name, double command)
{
    // Basic range validation (can be extended with joint-specific limits)
    const double MAX_JOINT_ANGLE = 3.14159;  // ±180 degrees
    
    if (std::abs(command) > MAX_JOINT_ANGLE)
    {
        ROS_WARN("Command for joint %s (%.3f) exceeds maximum angle limit (±%.3f)", 
                 joint_name.c_str(), command, MAX_JOINT_ANGLE);
        return false;
    }
    
    if (std::isnan(command) || std::isinf(command))
    {
        ROS_WARN("Invalid command for joint %s: %f", joint_name.c_str(), command);
        return false;
    }
    
    return true;
}

} // namespace g1_control 