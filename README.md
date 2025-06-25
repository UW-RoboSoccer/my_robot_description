ROS 2 pacakge that defines the robot for simulation using MuJoCo. 

mujoco.launch.py:
1. loads urdf and coverts to robot description
2. starts mujoco_ros2_control node
3. spawns multiple controllers for different robot functions

URDF → XACRO Processing → Robot Description → MuJoCo Engine
                                                    ↓
Controllers ← ROS 2 Control ← MuJoCo ROS 2 Control ← Physics Simulation
TSID Controller Node
    ↓ (publishes joint commands)
/robosoccer/joint_commands (Float64MultiArray)
    ↓
TSID to Trajectory Bridge
    ↓ (converts to trajectory)
/position_controller/joint_trajectory (JointTrajectory)
    ↓
ros2_control Position Controller
    ↓ (sends to hardware interface)
mujoco_ros2_control (MujocoSystem)
    ↓ (applies to simulation)
MuJoCo Physics Engine