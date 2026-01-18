#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    # Package path
    pkg_path = get_package_share_directory('my_robot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # File paths
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'my_world.sdf')
    slam_config = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_path, 'rviz', 'slam_view.rviz')
    
    # Process xacro
    robot_description_config = xacro.process_file(urdf_file)
    robot_description_xml = robot_description_config.toxml()
    
    # Launch argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Start Ignition Gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # ROS-Ignition Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # TF: odom -> base_footprint (from odometry)
    # For simulation, we publish static TF initially
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

        # Fix for Frame Mismatch
    # Replace 'ign_scan_frame' with whatever 'ros2 topic echo /scan' showed you
        # FIX FOR LIDAR FRAME MISMATCH
    # Replace 'my_robot/base_link/lidar_sensor' with the ACTUAL frame_id you found in Step 2
    # If you used 'lidar', put 'lidar' there.
    fix_lidar_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fix_lidar_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'my_robot/base_footprint/lidar_sensor'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Delayed actions
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])
    delayed_slam = TimerAction(period=5.0, actions=[slam_toolbox])
    delayed_rviz = TimerAction(period=6.0, actions=[rviz])
    
    return LaunchDescription([
        declare_use_sim_time,
        start_gazebo,
        robot_state_publisher,
        bridge,
        static_tf_odom,
        delayed_spawn,
        delayed_slam,
        delayed_rviz,
       fix_lidar_frame,
    ])