import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # -----------------------------
    # Robot Description (Xacro)
    # -----------------------------
    pkg_robot = get_package_share_directory("Assm_description")
    xacro_file = os.path.join(pkg_robot, "urdf", "Assm.xacro")

    robot_description = Command([
        "xacro",
        " ",
        xacro_file
    ])

    # -----------------------------
    # Fix Gazebo plugin path
    # -----------------------------
    gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=f"{os.environ.get('GAZEBO_PLUGIN_PATH', '')}:/opt/ros/rolling/lib"
    )

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    show_gui = LaunchConfiguration('gui')

    # -----------------------------
    # Gazebo
    # -----------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "verbose": "true"
        }.items(),
    )

    # -----------------------------
    # Spawn Entity
    # -----------------------------
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "Assm",
            "-z", "0.1"
        ],
        output="screen",
    )

    # -----------------------------
    # Controllers
    # -----------------------------
    joint_state_broadcaster = Node(
        condition=IfCondition(show_gui),
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    leg_controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[f"leg{i}_controller"],
            output="screen",
        )
        for i in range(1, 5)
    ]

    # -----------------------------
    # Timed Start for controllers
    # -----------------------------
    delays = [
        TimerAction(period=5.0, actions=[joint_state_broadcaster]),
        TimerAction(period=7.0, actions=[leg_controllers[0]]),
        TimerAction(period=8.0, actions=[leg_controllers[1]]),
        TimerAction(period=9.0, actions=[leg_controllers[2]]),
        TimerAction(period=10.0, actions=[leg_controllers[3]]),
    ]

    # -----------------------------
    # Leg initial positions as ROS2 topic pubs
    # -----------------------------
    leg_positions = [
        [0.00, 0.55, 2.79],  # Leg 1
        [0.00, 0.01, -0.30],  # Leg 2
        [0.00, 0.79, 0.43],  # Leg 3
        [0.00, -0.02, 1.28],  # Leg 4
    ]

    leg_init_timers = []
    for i, positions in enumerate(leg_positions, start=1):
        # Format the message as a proper YAML string
        msg = (
            "{joint_names: [Revolute_Coxa_" + str(i) + ", "
            "Revolute_Femur_" + str(i) + ", "
            "Revolute_Tibia_" + str(i) + "], "
            "points: [{positions: [" + ", ".join(map(str, positions)) + "], "
            "time_from_start: {sec: 2}}]}"
        )
        
        cmd = ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub",
                f"/leg{i}_controller/joint_trajectory",
                "trajectory_msgs/msg/JointTrajectory",
                msg,
                "--once"
            ],
            shell=False
        )
        # stagger slightly after controller spawner
        timer = TimerAction(period=11.0 + i, actions=[cmd])
        leg_init_timers.append(timer)

    # -----------------------------
    # Reset Simulation after all joints initialized
    # -----------------------------
    reset_simulation = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/reset_simulation', 'std_srvs/srv/Empty'],
        shell=False
    )
    reset_timer = TimerAction(period=20.0, actions=[reset_simulation])

    # -----------------------------
    # Optional Joint State Publisher GUI
    # -----------------------------
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # -----------------------------
    # Final LaunchDescription
    # -----------------------------
    return LaunchDescription([
        gazebo_plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        *delays,           # controller spawners
        *leg_init_timers,  # run initial leg positions
        reset_timer,       # reset simulation after init
        # joint_state_publisher_gui_node,
    ])