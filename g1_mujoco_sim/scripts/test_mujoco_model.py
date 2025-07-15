#!/usr/bin/env python3

import mujoco
import mujoco.viewer
import time

# Load the model
model = mujoco.MjModel.from_xml_path('src/g1_description/models/g1_humanoid.xml')
data = mujoco.MjData(model)

# Set initial joint positions for standing pose
standing_pose = {
    "left_hip_yaw_joint": 0.0,
    "left_hip_roll_joint": 0.0,
    "left_hip_pitch_joint": -0.3,
    "left_knee_joint": 0.6,
    "left_ankle_pitch_joint": -0.3,
    "left_ankle_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_hip_roll_joint": 0.0,
    "right_hip_pitch_joint": -0.3,
    "right_knee_joint": 0.6,
    "right_ankle_pitch_joint": -0.3,
    "right_ankle_roll_joint": 0.0,
    "waist_yaw_joint": 0.0,
    "left_shoulder_pitch_joint": 0.3,
    "left_shoulder_roll_joint": 0.2,
    "left_elbow_joint": -0.5,
    "left_wrist_joint": 0.0,
    "right_shoulder_pitch_joint": 0.3,
    "right_shoulder_roll_joint": -0.2,
    "right_elbow_joint": -0.5,
    "right_wrist_joint": 0.0,
    "head_yaw_joint": 0.0
}

# Apply standing pose
for joint_name, value in standing_pose.items():
    joint_id = model.joint(joint_name).id
    qpos_idx = model.jnt_qposadr[joint_id]
    data.qpos[qpos_idx] = value

# Create viewer
viewer = mujoco.viewer.launch_passive(model, data)

# Run simulation
print("G1 Humanoid Robot Model Loaded Successfully!")
print("Close the viewer window to exit.")

while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()
    time.sleep(0.01)

print("Viewer closed.") 