#!/usr/bin/env python3

import rospy
import numpy as np
import mujoco
import mujoco.viewer
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from visualization_msgs.msg import Marker
import tf2_ros
from geometry_msgs.msg import TransformStamped
import os
import time


class MujocoSimPassiveNode:
    def __init__(self):
        rospy.init_node('mujoco_sim_node', anonymous=True)
        
        # Get model path from parameter
        model_path = rospy.get_param('~model_path', 
                                     os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                                 '../g1_description/models/g1_humanoid.xml'))
        
        # Load MuJoCo model
        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            rospy.loginfo(f"Successfully loaded MuJoCo model from {model_path}")
            
            # Initialize robot to stable keyframe pose if available
            keyframe_name = "home"  
            keyframe_id = None
            for i in range(self.model.nkey):
                if self.model.key(i).name == keyframe_name:
                    keyframe_id = i
                    break
            
            if keyframe_id is not None:
                mujoco.mj_resetDataKeyframe(self.model, self.data, keyframe_id)
                mujoco.mj_forward(self.model, self.data)  # Update derived quantities
                rospy.loginfo(f"Initialized robot to '{keyframe_name}' keyframe pose")
            else:
                # If no keyframe found, use default initialization
                mujoco.mj_resetData(self.model, self.data)
                mujoco.mj_forward(self.model, self.data)  # Update derived quantities
                rospy.logwarn(f"Keyframe '{keyframe_name}' not found, using default initialization")
                
        except Exception as e:
            rospy.logerr(f"Failed to load MuJoCo model: {e}")
            return
        
        # Initialize joint names and indices for official G1 model (29 DOF)
        self.joint_names = [
            # Left leg (6 DOF)
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            # Right leg (6 DOF)
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            # Waist (3 DOF)
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            # Left arm (7 DOF)
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            # Right arm (7 DOF)
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        ]
        
        # Map joint names to MuJoCo indices
        self.joint_indices = {}
        for name in self.joint_names:
            try:
                self.joint_indices[name] = self.model.joint(name).id
            except:
                rospy.logwarn(f"Joint {name} not found in model")
        
        # Publishers (selective publishing based on parameters)
        self.publish_joint_states = rospy.get_param('~publish_joint_states', False)
        self.publish_base_pose = rospy.get_param('~publish_base_pose', True)
        
        if self.publish_joint_states:
            self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
            rospy.loginfo("Joint states publishing ENABLED")
        else:
            rospy.loginfo("Joint states publishing DISABLED (passive mode)")
            
        if self.publish_base_pose:
            self.base_pose_pub = rospy.Publisher('/base_pose', PoseStamped, queue_size=10)
            self.base_twist_pub = rospy.Publisher('/base_twist', TwistStamped, queue_size=10)
            # TF broadcaster
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        self.joint_cmd_sub = rospy.Subscriber('/joint_commands', Float64MultiArray, 
                                            self.joint_command_callback, queue_size=1)
        
        # Control variables
        self.joint_commands = np.zeros(len(self.joint_names))
        self.command_lock = threading.Lock()
        
        # Simulation parameters
        self.sim_rate = rospy.get_param('~sim_rate', 1000)  # Hz
        self.publish_rate = rospy.get_param('~publish_rate', 100)  # Hz
        self.enable_visualization = rospy.get_param('~enable_visualization', True)
        
        # Start simulation in separate thread
        self.sim_thread = threading.Thread(target=self.simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        
        # Start publisher timer (only if needed)
        if self.publish_joint_states or self.publish_base_pose:
            self.publish_timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), 
                                           self.publish_callback)
        
        rospy.loginfo("MuJoCo simulation node started")
        rospy.loginfo(f"Listening for joint commands on /joint_commands")
        rospy.loginfo(f"Simulation rate: {self.sim_rate} Hz")
    
    def joint_command_callback(self, msg):
        """Handle joint command messages from joint_gui_controller"""
        if len(msg.data) != len(self.joint_names):
            rospy.logwarn(f"Received {len(msg.data)} commands but expected {len(self.joint_names)}")
            return
        
        with self.command_lock:
            self.joint_commands = np.array(msg.data)
        
        rospy.logdebug(f"Received joint commands: {self.joint_commands[:5]}...")  # Log first 5 values
    
    def simulation_loop(self):
        """Main simulation loop running in separate thread"""
        if self.enable_visualization:
            viewer = mujoco.viewer.launch_passive(self.model, self.data)
            rospy.loginfo("MuJoCo viewer launched")
        
        dt = 1.0 / self.sim_rate
        
        while not rospy.is_shutdown():
            start_time = time.time()
            
            # Apply control commands to actuators
            with self.command_lock:
                for i, name in enumerate(self.joint_names):
                    if name in self.joint_indices:
                        # For the official G1 model, actuator names match joint names
                        try:
                            actuator_id = self.model.actuator(name).id
                            self.data.ctrl[actuator_id] = self.joint_commands[i]
                        except Exception as e:
                            # Fallback: try without "_joint" suffix
                            try:
                                actuator_name = name.replace("_joint", "")
                                actuator_id = self.model.actuator(actuator_name).id
                                self.data.ctrl[actuator_id] = self.joint_commands[i]
                            except:
                                pass
            
            # Step simulation
            mujoco.mj_step(self.model, self.data)
            
            # Update viewer if enabled
            if self.enable_visualization and viewer.is_running():
                viewer.sync()
            
            # Sleep to maintain rate
            elapsed = time.time() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)
    
    def publish_callback(self, event):
        """Publish robot state (selective based on parameters)"""
        
        # Publish joint states only if enabled
        if self.publish_joint_states:
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.header.frame_id = "world"
            
            positions = []
            velocities = []
            efforts = []
            
            for name in self.joint_names:
                if name in self.joint_indices:
                    joint_id = self.joint_indices[name]
                    qpos_idx = self.model.jnt_qposadr[joint_id]
                    qvel_idx = self.model.jnt_dofadr[joint_id]
                    
                    positions.append(self.data.qpos[qpos_idx])
                    velocities.append(self.data.qvel[qvel_idx])
                    
                    # Get actuator force if available
                    try:
                        actuator_id = self.model.actuator(name).id
                        efforts.append(self.data.actuator_force[actuator_id])
                    except:
                        # Fallback: try without "_joint" suffix
                        try:
                            actuator_name = name.replace("_joint", "")
                            actuator_id = self.model.actuator(actuator_name).id
                            efforts.append(self.data.actuator_force[actuator_id])
                        except:
                            efforts.append(0.0)
                else:
                    positions.append(0.0)
                    velocities.append(0.0)
                    efforts.append(0.0)
            
            joint_state.name = self.joint_names
            joint_state.position = positions
            joint_state.velocity = velocities
            joint_state.effort = efforts
            
            self.joint_state_pub.publish(joint_state)
        
        # Publish base pose and twist if enabled
        if self.publish_base_pose:
            # Publish base pose (using pelvis as main body in official G1 model)
            base_id = self.model.body("pelvis").id
            base_pos = self.data.xpos[base_id]
            base_quat = self.data.xquat[base_id]
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = base_pos[0]
            pose_msg.pose.position.y = base_pos[1]
            pose_msg.pose.position.z = base_pos[2]
            pose_msg.pose.orientation.w = base_quat[0]
            pose_msg.pose.orientation.x = base_quat[1]
            pose_msg.pose.orientation.y = base_quat[2]
            pose_msg.pose.orientation.z = base_quat[3]
            
            self.base_pose_pub.publish(pose_msg)
            
            # Publish base twist
            base_vel = self.data.cvel[base_id]
            twist_msg = TwistStamped()
            twist_msg.header = pose_msg.header
            twist_msg.twist.linear.x = base_vel[3]
            twist_msg.twist.linear.y = base_vel[4]
            twist_msg.twist.linear.z = base_vel[5]
            twist_msg.twist.angular.x = base_vel[0]
            twist_msg.twist.angular.y = base_vel[1]
            twist_msg.twist.angular.z = base_vel[2]
            
            self.base_twist_pub.publish(twist_msg)
            
            # Broadcast TF (keeping base_link as ROS standard frame name)
            t = TransformStamped()
            t.header = pose_msg.header
            t.child_frame_id = "base_link"
            t.transform.translation.x = base_pos[0]
            t.transform.translation.y = base_pos[1]
            t.transform.translation.z = base_pos[2]
            t.transform.rotation = pose_msg.pose.orientation
            
            self.tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    try:
        node = MujocoSimPassiveNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 