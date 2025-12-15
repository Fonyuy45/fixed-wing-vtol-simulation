import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('fixar_007_description')
    
    # Paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'fuselage.xacro')
    
    # Set Gazebo resource path to find meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_share
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', default_model_path]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
          'gz_args': os.path.join(pkg_share, 'worlds', 'empty_with_sensors.sdf') + ' -r'
        }.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'fixar_007',
            '-z', '1.0'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge


    bridge = Node(
	    package='ros_gz_bridge',
	    executable='parameter_bridge',
	    arguments=[
		'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
		'/fixar_007/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
		'/fixar_007/altimeter@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
		'/fixar_007/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
		'/fixar_007/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',  # ADD THIS LINE
		'/fixar_007/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
		'/fixar_007/motor_front_right/cmd@std_msgs/msg/Float64]gz.msgs.Double',
		'/fixar_007/motor_front_left/cmd@std_msgs/msg/Float64]gz.msgs.Double',
		'/fixar_007/motor_rear_right/cmd@std_msgs/msg/Float64]gz.msgs.Double',
		'/fixar_007/motor_rear_left/cmd@std_msgs/msg/Float64]gz.msgs.Double',
	    ],
	    output='screen'
	)
    
    return LaunchDescription([
        gz_resource_path,
        declare_use_sim_time,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])
