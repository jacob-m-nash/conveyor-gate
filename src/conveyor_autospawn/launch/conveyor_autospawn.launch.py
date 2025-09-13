from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument("speed", default_value="50"),
        DeclareLaunchArgument("y", default_value="-0.5"),
        DeclareLaunchArgument("z", default_value="0.76"),
        DeclareLaunchArgument('x_min', default_value='-0.05'),
        DeclareLaunchArgument('x_max', default_value='0.05'),
        DeclareLaunchArgument('min_interval_s', default_value='2.0'),
        DeclareLaunchArgument('max_interval_s', default_value='6.0'),
      ]
    
    conv_pkg_share = get_package_share_directory("conveyorbelt_gazebo")
    conv_launch = os.path.join(conv_pkg_share, "launch", "conveyorbelt.launch.py")
    include_world = IncludeLaunchDescription(PythonLaunchDescriptionSource(conv_launch))

    autospawn_node = Node(
        package= "conveyor_autospawn",
        executable="conveyor_autospawn_node",
        name="conveyor_autospawn",
        output = "screen",
        parameters=[{
            'speed': LaunchConfiguration('speed'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'x_min': LaunchConfiguration('x_min'),
            'x_max': LaunchConfiguration('x_max'),
            'min_interval_s': LaunchConfiguration('min_interval_s'),
            'max_interval_s': LaunchConfiguration('max_interval_s'),
        }]
    )
    return LaunchDescription(declare_args + [include_world,autospawn_node])