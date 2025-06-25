from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindPackageShare, PathJoinSubstitution

def generate_launch_description():
    """
    This launch file is used to convert the robot's URDF description into an MJCF format.
    
    It uses the `mujoco_ros` package's converter node to perform the conversion. The resulting
    MJCF file is saved in the `v1_mjcf` directory. This approach ensures that the MJCF model
    is always in sync with the URDF, and it avoids the errors that can be introduced by
    manual or external conversion tools.
    
    To run this launch file, use the following command:
    ros2 launch my_robot_description convert_urdf_to_mjcf.launch.py
    """
    
    # Define the input and output file paths
    urdf_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'v1_urdf',
        'robot.urdf'
    ])
    
    mjcf_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'v1_mjcf',
        'robot.xml'
    ])

    # Create the conversion command
    conversion_command = ExecuteProcess(
        cmd=[
            'mujoco_ros_converter',
            '--urdf-path', urdf_file,
            '--mjcf-path', mjcf_file,
            '--mesh-dir', PathJoinSubstitution([FindPackageShare('my_robot_description'), 'v1_meshes']),
        ],
        output='screen'
    )

    return LaunchDescription([
        conversion_command
    ]) 