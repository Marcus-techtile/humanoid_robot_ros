#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <map>
#include <vector>
#include <string>

namespace g1_control
{

class JointController
{
public:
    /**
     * @brief Constructor for JointController
     */
    JointController();
    
    /**
     * @brief Destructor for JointController
     */
    ~JointController() = default;

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber joint_command_sub_;
    ros::Publisher joint_cmd_pub_;
    
    // Joint names in the order expected by the robot controller
    std::vector<std::string> robot_joint_names_;
    
    // Map to store the latest joint positions from command source
    std::map<std::string, double> command_joint_positions_;
    
    // Control parameters
    double control_rate_;
    ros::Timer control_timer_;
    
    // Safety limits
    double max_joint_velocity_;
    double position_tolerance_;
    
    // Topic names (configurable)
    std::string joint_command_topic_;
    std::string joint_output_topic_;
    
    // Control mode
    bool direct_mode_;  // If true, directly forward commands; if false, use internal control loop
    
    /**
     * @brief Initialize the robot joint names array
     */
    void initializeJointNames();
    
    /**
     * @brief Load parameters from ROS parameter server
     */
    void loadParameters();
    
    /**
     * @brief Callback for joint state messages from any command source
     * @param msg Joint state message
     */
    void jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg);
    
    /**
     * @brief Callback for direct joint command array (alternative input)
     * @param msg Float64MultiArray message with joint commands
     */
    void directCommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    
    /**
     * @brief Control loop callback to publish joint commands
     * @param event Timer event
     */
    void controlCallback(const ros::TimerEvent& event);
    
    /**
     * @brief Validate joint command for safety
     * @param joint_name Name of the joint
     * @param command Command value
     * @return True if command is safe
     */
    bool validateJointCommand(const std::string& joint_name, double command);
};

} // namespace g1_control

#endif // JOINT_CONTROLLER_H 