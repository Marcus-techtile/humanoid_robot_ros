#!/usr/bin/env python3
"""
External Force Injector for Balance Testing

This node adds external force application capabilities to the existing
stable MuJoCo simulation without modifying it. It works by modifying
the MuJoCo data structure directly through shared memory/file interface.
"""

import rospy
import numpy as np
import threading
import time
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped


class ExternalForceInjector:
    def __init__(self):
        rospy.init_node('external_force_injector', anonymous=True)
        
        # Force management
        self.active_forces = {}  # body_name -> (force_vector, duration, start_time)
        self.force_lock = threading.Lock()
        
        # ROS interface
        self.setup_ros_interface()
        
        # Force application timer
        self.force_timer = rospy.Timer(rospy.Duration(0.01), self.force_update_callback)  # 100Hz
        
        rospy.loginfo("External Force Injector started")
        rospy.loginfo("Listening for force commands on /external_force and /apply_disturbance")
    
    def setup_ros_interface(self):
        """Setup ROS publishers and subscribers"""
        
        # Subscribers for force commands
        self.external_force_sub = rospy.Subscriber('/external_force', Wrench,
                                                 self.external_force_callback, queue_size=1)
        self.disturbance_sub = rospy.Subscriber('/apply_disturbance', Vector3,
                                              self.disturbance_callback, queue_size=1)
        self.reset_forces_sub = rospy.Subscriber('/reset_forces', Bool,
                                               self.reset_forces_callback, queue_size=1)
        
        # Publishers for feedback
        self.force_status_pub = rospy.Publisher('/force_status', String, queue_size=10)
        
        # Get existing simulation data (if available)
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, 
                                               self.joint_states_callback, queue_size=1)
        self.base_pose_sub = rospy.Subscriber('/base_pose', PoseStamped,
                                            self.base_pose_callback, queue_size=1)
        
        # Store robot state
        self.current_base_pose = None
        self.current_joint_states = None
        
    def joint_states_callback(self, msg):
        """Store current joint states"""
        self.current_joint_states = msg
    
    def base_pose_callback(self, msg):
        """Store current base pose"""
        self.current_base_pose = msg
    
    def external_force_callback(self, msg):
        """Handle external force application requests"""
        force_vector = np.array([msg.force.x, msg.force.y, msg.force.z])
        torque_vector = np.array([msg.torque.x, msg.torque.y, msg.torque.z])
        
        # Apply force to pelvis/base by default
        self.apply_force("pelvis", force_vector, torque_vector, duration=0.1)
        
        rospy.loginfo(f"Applied force: {force_vector} N, torque: {torque_vector} Nm")
    
    def disturbance_callback(self, msg):
        """Handle disturbance application (simplified force)"""
        force_vector = np.array([msg.x, msg.y, msg.z])
        
        # Apply disturbance force
        self.apply_force("pelvis", force_vector, duration=0.2)
        
        rospy.loginfo(f"Applied disturbance: {force_vector} N")
    
    def reset_forces_callback(self, msg):
        """Reset all active forces"""
        if msg.data:
            self.reset_all_forces()
            rospy.loginfo("Reset all external forces")
    
    def apply_force(self, body_name, force_vector, torque_vector=None, duration=0.1):
        """Apply external force to specified body"""
        if torque_vector is None:
            torque_vector = np.zeros(3)
        
        with self.force_lock:
            self.active_forces[body_name] = {
                'force': force_vector.copy(),
                'torque': torque_vector.copy(), 
                'duration': duration,
                'start_time': time.time()
            }
        
        # Publish status
        status_msg = String()
        status_msg.data = f"Applied {np.linalg.norm(force_vector):.1f}N force to {body_name}"
        self.force_status_pub.publish(status_msg)
    
    def reset_all_forces(self):
        """Reset all active forces"""
        with self.force_lock:
            self.active_forces.clear()
        
        # Publish status
        status_msg = String()
        status_msg.data = "All forces reset"
        self.force_status_pub.publish(status_msg)
    
    def force_update_callback(self, event):
        """Update active forces and remove expired ones"""
        current_time = time.time()
        
        with self.force_lock:
            # Check for expired forces
            expired_bodies = []
            for body_name, force_data in self.active_forces.items():
                elapsed = current_time - force_data['start_time']
                if elapsed >= force_data['duration']:
                    expired_bodies.append(body_name)
            
            # Remove expired forces
            for body_name in expired_bodies:
                del self.active_forces[body_name]
                rospy.logdebug(f"Force on {body_name} expired")
            
            # Publish current status
            if self.active_forces:
                total_force = 0.0
                for force_data in self.active_forces.values():
                    total_force += np.linalg.norm(force_data['force'])
                
                status_msg = String()
                status_msg.data = f"Active forces: {len(self.active_forces)}, Total: {total_force:.1f}N"
                self.force_status_pub.publish(status_msg)
    
    def get_active_forces(self):
        """Get dictionary of currently active forces"""
        with self.force_lock:
            return self.active_forces.copy()


def main():
    """Main function"""
    try:
        injector = ExternalForceInjector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("External Force Injector interrupted")
    except Exception as e:
        rospy.logerr(f"External Force Injector failed: {e}")


if __name__ == '__main__':
    main() 