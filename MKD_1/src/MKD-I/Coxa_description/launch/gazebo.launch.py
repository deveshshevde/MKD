# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration,PythonExpression
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# import xacro
# from os.path import join

# def generate_launch_description():

#     # Package Directories
#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
#     pkg_ros_gz_rbot = get_package_share_directory('Coxa_description')

#     # Parse robot description from xacro
#     robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'Coxa.xacro')
#     ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    
#     robot_description_config = xacro.process_file(
#         robot_description_file
#     )
#     robot_description = {'robot_description': robot_description_config.toxml()}

#     # Start Robot state publisher
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='both',
#         parameters=[robot_description],
#     )

#     ros2_control_node = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[
#             robot_description,  # your xacro URDF
#             "/home/dd/Documents/MKD_1/src/MKD-I/Coxa_description/config/controls.yaml",
#         ],
#         output='screen'
#     )

#     # Start Gazebo Sim
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
#         launch_arguments={
#             "gz_args" : '-r -v 4 empty.sdf'
#         }.items()
#     )

#     # Spawn Robot in Gazebo   
#     spawn = Node(
#         package='ros_gz_sim',
#         executable='create',
#         arguments=[
#             "-topic", "/robot_description",
#             "-name", "Coxa",
#             "-allow_renaming", "true",
#             "-z", "0.32",
#             "-x", "0.0",
#             "-y", "0.0",
#             "-Y", "0.0"
#         ],            
#         output='screen',
#     )

#     # Bridge ROS topics and Gazebo messages for establishing communication
#     start_gazebo_ros_bridge_cmd = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         parameters=[{
#           'config_file': ros_gz_bridge_config,
#         }],
#         output='screen'
#       )      


#     return LaunchDescription(
#         [
#             # Nodes and Launches
#             gazebo,
#             spawn,
#             start_gazebo_ros_bridge_cmd,
#             robot_state_publisher,
#             ros2_control_node, 
#         ]
#     )

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('Coxa_description')

    # Paths
    robot_description_file = join(pkg_ros_gz_rbot, 'urdf', 'Coxa.xacro')
    ros_gz_bridge_config = join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    ros2_control_config = join(pkg_ros_gz_rbot, 'config', 'controls.yaml')

    # Process URDF/XACRO
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ROS 2 Control Node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            ros2_control_config
        ],
        output='screen'
    )

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'Coxa',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.32',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Start ROS-Gazebo bridge
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn,
        start_gazebo_ros_bridge_cmd,
        robot_state_publisher,
        ros2_control_node
    ])

