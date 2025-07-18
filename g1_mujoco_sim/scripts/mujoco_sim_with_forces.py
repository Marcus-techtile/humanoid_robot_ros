#!/usr/bin/env python3

import rospy
import numpy as np
import mujoco
import mujoco.viewer
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, String
from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench, Vector3
from visualization_msgs.msg import Marker
import tf2_ros
from geometry_msgs.msg import TransformStamped
import os
import time


class MujocoSimWithForcesNode:
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
        
        # External force management (NEW)
        self.external_forces = {}
        self.force_duration = {}
        self.force_lock = threading.Lock()
        self.enable_external_forces = rospy.get_param('~enable_external_forces', True)
        
        # Publishers (selective publishing based on parameters)
        self.publish_joint_states = rospy.get_param('~publish_joint_states', True)  # Default True for testing
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
        
        # External force subscribers (NEW)
        if self.enable_external_forces:
            self.external_force_sub = rospy.Subscriber('/external_force', Wrench,
                                                     self.external_force_callback, queue_size=1)
            self.disturbance_sub = rospy.Subscriber('/apply_disturbance', Vector3,
                                                  self.disturbance_callback, queue_size=1)
            self.reset_forces_sub = rospy.Subscriber('/reset_forces', Bool,
                                                   self.reset_forces_callback, queue_size=1)
            # Force status publisher
            self.force_status_pub = rospy.Publisher('/force_status', String, queue_size=10)
            rospy.loginfo("External force capability ENABLED")
        
        # Control variables
        self.joint_commands = np.zeros(len(self.joint_names))
        self.command_lock = threading.Lock()
        self.has_new_commands = False
        
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
        
        rospy.loginfo("MuJoCo simulation with forces node started")
        rospy.loginfo(f"Listening for joint commands on /joint_commands")
        rospy.loginfo(f"Simulation rate: {self.sim_rate} Hz")
    
    def joint_command_callback(self, msg):
        """Handle joint command messages from joint_gui_controller"""
        if len(msg.data) != len(self.joint_names):
            rospy.logwarn(f"Received {len(msg.data)} commands but expected {len(self.joint_names)}")
            return
        
        with self.command_lock:
            self.joint_commands = np.array(msg.data)
            self.has_new_commands = True
        
        rospy.logdebug(f"Received joint commands: {self.joint_commands[:5]}...")  # Log first 5 values
    
    # NEW: External force callback methods
    def external_force_callback(self, msg):
        """Handle external force application requests"""
        if not self.enable_external_forces:
            return
        
        force_vector = np.array([msg.force.x, msg.force.y, msg.force.z])
        torque_vector = np.array([msg.torque.x, msg.torque.y, msg.torque.z])
        
        # Apply force to pelvis by default
        self.apply_external_force("pelvis", force_vector, torque_vector, duration=0.1)
        
        rospy.loginfo(f"Applied force: {force_vector} N, torque: {torque_vector} Nm")
    
    def disturbance_callback(self, msg):
        """Handle disturbance application (simplified force)"""
        if not self.enable_external_forces:
            return
        
        force_vector = np.array([msg.x, msg.y, msg.z])
        
        # Apply disturbance force
        self.apply_external_force("pelvis", force_vector, duration=0.2)
        
        rospy.loginfo(f"Applied disturbance: {force_vector} N")
    
    def reset_forces_callback(self, msg):
        """Reset all external forces"""
        if msg.data and self.enable_external_forces:
            self.reset_all_forces()
            rospy.loginfo("Reset all external forces")
    
    def apply_external_force(self, body_name, force_vector, torque_vector=None, duration=0.1):
        """Apply external force to specified body"""
        try:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id < 0:
                rospy.logwarn(f"Body {body_name} not found")
                return
            
            if torque_vector is None:
                torque_vector = np.zeros(3)
            
            with self.force_lock:
                # Store force and torque
                self.external_forces[body_id] = np.concatenate([force_vector, torque_vector])
                self.force_duration[body_id] = duration
            
            # Publish status
            if hasattr(self, 'force_status_pub'):
                status_msg = String()
                status_msg.data = f"Applied {np.linalg.norm(force_vector):.1f}N force to {body_name}"
                self.force_status_pub.publish(status_msg)
            
            rospy.loginfo(f"Applied force {force_vector} to {body_name} for {duration}s")
            
        except Exception as e:
            rospy.logwarn(f"Failed to apply force to {body_name}: {e}")
    
    def reset_all_forces(self):
        """Reset all external forces"""
        with self.force_lock:
            self.external_forces.clear()
            self.force_duration.clear()
        
        # Clear MuJoCo force arrays
        for i in range(self.model.nbody):
            self.data.xfrc_applied[i][:] = 0
        
        if hasattr(self, 'force_status_pub'):
            status_msg = String()
            status_msg.data = "All forces reset"
            self.force_status_pub.publish(status_msg)
    
    def update_external_forces(self, dt):
        """Update external forces in simulation"""
        if not self.enable_external_forces:
            return
            
        with self.force_lock:
            # Apply current forces
            for body_id, force_torque in self.external_forces.items():
                if body_id < self.model.nbody:
                    self.data.xfrc_applied[body_id][:] = force_torque
            
            # Update durations and remove expired forces
            expired_bodies = []
            for body_id in self.force_duration:
                self.force_duration[body_id] -= dt
                if self.force_duration[body_id] <= 0:
                    expired_bodies.append(body_id)
            
            # Remove expired forces
            for body_id in expired_bodies:
                del self.external_forces[body_id]
                del self.force_duration[body_id]
                if body_id < self.model.nbody:
                    self.data.xfrc_applied[body_id][:] = 0
    
    def simulation_loop(self):
        """Main simulation loop running in separate thread"""
        if self.enable_visualization:
            viewer = mujoco.viewer.launch_passive(self.model, self.data)
            rospy.loginfo("MuJoCo viewer launched")
        
        dt = 1.0 / self.sim_rate
        last_time = time.time()
        
        while not rospy.is_shutdown():
            start_time = time.time()
            actual_dt = start_time - last_time
            last_time = start_time
            
            # Apply control commands to actuators only if new data is available
            with self.command_lock:
                if self.has_new_commands:
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
                    self.has_new_commands = False
            
            # NEW: Update external forces
            if self.enable_external_forces:
                self.update_external_forces(actual_dt)
            
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
        
        # Publish base pose and twist only if enabled
        if self.publish_base_pose:
            # Base pose (assuming freejoint for floating base)
            base_pose = PoseStamped()
            base_pose.header.stamp = rospy.Time.now()
            base_pose.header.frame_id = "world"
            
            # Position (first 3 elements of qpos)
            if len(self.data.qpos) >= 7:
                base_pose.pose.position.x = self.data.qpos[0]
                base_pose.pose.position.y = self.data.qpos[1]
                base_pose.pose.position.z = self.data.qpos[2]
                
                # Orientation (quaternion: w, x, y, z in qpos)
                base_pose.pose.orientation.w = self.data.qpos[3]
                base_pose.pose.orientation.x = self.data.qpos[4]
                base_pose.pose.orientation.y = self.data.qpos[5]
                base_pose.pose.orientation.z = self.data.qpos[6]
            
            self.base_pose_pub.publish(base_pose)
            
            # Base twist
            base_twist = TwistStamped()
            base_twist.header.stamp = rospy.Time.now()
            base_twist.header.frame_id = "world"
            
            if len(self.data.qvel) >= 6:
                # Linear velocity
                base_twist.twist.linear.x = self.data.qvel[0]
                base_twist.twist.linear.y = self.data.qvel[1] 
                base_twist.twist.linear.z = self.data.qvel[2]
                
                # Angular velocity
                base_twist.twist.angular.x = self.data.qvel[3]
                base_twist.twist.angular.y = self.data.qvel[4]
                base_twist.twist.angular.z = self.data.qvel[5]
            
            self.base_twist_pub.publish(base_twist)
            
            # Publish TF transform
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "base_link"
            t.transform.translation.x = base_pose.pose.position.x
            t.transform.translation.y = base_pose.pose.position.y
            t.transform.translation.z = base_pose.pose.position.z
            t.transform.rotation = base_pose.pose.orientation
            
            self.tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    try:
        node = MujocoSimWithForcesNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 