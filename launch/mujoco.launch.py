import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_path, 'v1_urdf', 'robot.urdf')
    mjcf_file = os.path.join(pkg_path, 'v1_mjcf', 'robot.xml')
    controller_yaml = os.path.join(pkg_path, 'config', 'controllers.yaml')

    robot_description_content = xacro.process_file(urdf_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    print("==== ROBOT DESCRIPTION ====")
    print(robot_description_content[:500])  # first 500 chars

    mujoco_model_path = os.path.join(pkg_path, 'v1_mjcf', 'robot.xml')

    controller_manager_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_yaml,
            {'mujoco_model_path': mujoco_model_path}
        ]
    )

    return LaunchDescription([
        controller_manager_node,
        # Start the controllers
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controller"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["imu_sensor_broadcaster"],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["torso_test_fts_broadcaster"],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["right_fts_broadcaster"],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["left_fts_broadcaster"],
            output="screen"
        )
    ])
