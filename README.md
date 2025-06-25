ROS 2 pacakge that defines the robot for simulation using MuJoCo. 

mujoco.launch.py:
1. loads urdf and coverts to robot description
2. starts mujoco_ros2_control node
3. spawns multiple controllers for different robot functions

URDF → XACRO Processing → Robot Description → MuJoCo Engine
                                                    ↓
Controllers ← ROS 2 Control ← MuJoCo ROS 2 Control ← Physics Simulation
