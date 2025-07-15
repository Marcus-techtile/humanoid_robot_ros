#!/usr/bin/env python3

import mujoco
import numpy as np

# Load the MuJoCo model
model_path = "src/g1_description/scene_mjx.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print(f"Model loaded from: {model_path}")
print(f"Number of actuators: {model.nu}")
print(f"Number of joints: {model.nv}")
print()

# List all actuators
print("=== ACTUATORS ===")
for i in range(model.nu):
    actuator = model.actuator(i)
    print(f"Actuator {i}: {actuator.name}")

print()

# List all joints
print("=== JOINTS ===")
for i in range(model.nv):
    if i < model.njnt:
        joint = model.joint(i)
        print(f"Joint {i}: {joint.name}")
    else:
        print(f"Joint {i}: [DOF only, no physical joint]")

print()

# Test joint to actuator mapping
joint_names = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
]

print("=== JOINT TO ACTUATOR MAPPING ===")
for name in joint_names:
    try:
        actuator_id = model.actuator(name).id
        print(f"✓ {name} -> actuator {actuator_id}")
    except:
        # Try without "_joint" suffix
        try:
            actuator_name = name.replace("_joint", "")
            actuator_id = model.actuator(actuator_name).id
            print(f"✓ {name} -> actuator '{actuator_name}' (id {actuator_id})")
        except:
            print(f"✗ {name} -> NOT FOUND")

print()

# Check if actuators are position or torque controlled
print("=== ACTUATOR TYPES ===")
for i in range(model.nu):
    actuator = model.actuator(i)
    print(f"Actuator {i} ({actuator.name}): gainprm={model.actuator_gainprm[i]}, biasprm={model.actuator_biasprm[i]}") 