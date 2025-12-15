import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('fixar_007_description')
    
    # Path to URDF/xacro file
    default_model_path = os.path.join(pkg_share, 'urdf', 'fuselage.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')
    
    # Declare arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    
    # Get launch configurations
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rvizconfig')
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model])}]
    )
    
    # Joint State Publisher GUI node
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui)
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )
    
    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
