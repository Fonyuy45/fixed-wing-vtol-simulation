#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('fixar_007_description')
    
    # 1. World File
    world_file = os.path.join(pkg_share, 'worlds', 'empty_with_sensors.sdf')
    
    # 2. Model SDF Path
    sdf_file = os.path.join(pkg_share, 'models', 'fixar_007', 'model.sdf') 
    
    # 3. Resource Path
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_share
    )
    
    # 4. Start Gazebo (Fortress)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': world_file + ' -r'
        }.items()
    )
    
    # 5. Spawn the Drone
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file, 
            '-name', 'fixar_007',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )
    
    # 6. ROS-Gazebo Bridge (UNIFIED ACTUATORS ARRAY)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # Sensors
            '/fixar_007/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/fixar_007/air_pressure@sensor_msgs/msg/FluidPressure[ignition.msgs.FluidPressure',
            '/fixar_007/magnetometer@sensor_msgs/msg/MagneticField[ignition.msgs.Magnetometer',
            '/fixar_007/gps@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/fixar_007/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            
            # UNIFIED MOTOR COMMAND (Single array topic for all 4 motors)
            '/fixar_007/command/motor_speed@actuator_msgs/msg/Actuators]ignition.msgs.Actuators',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        set_model_path,
        gazebo,
        spawn_entity,
        bridge
    ])
