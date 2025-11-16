from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Autospawn args
    speed_arg        = DeclareLaunchArgument("speed", default_value="50")
    y_arg            = DeclareLaunchArgument("y", default_value="-0.5")
    z_arg            = DeclareLaunchArgument("z", default_value="0.76")
    x_min_arg        = DeclareLaunchArgument("x_min", default_value="-0.05")
    x_max_arg        = DeclareLaunchArgument("x_max", default_value="0.05")
    min_int_arg      = DeclareLaunchArgument("min_interval_s", default_value="2.0")
    max_int_arg      = DeclareLaunchArgument("max_interval_s", default_value="6.0")

    image_topic_arg  = DeclareLaunchArgument("image_topic", default_value="/camera/image_raw")
    min_frac_arg     = DeclareLaunchArgument("min_fraction", default_value="0.002")

    show_view_arg    = DeclareLaunchArgument("show_view", default_value="false")

    conv_share = get_package_share_directory("conveyorbelt_gazebo")
    conv_launch = os.path.join(conv_share, "launch", "conveyorbelt.launch.py")
    include_world = IncludeLaunchDescription(PythonLaunchDescriptionSource(conv_launch))


    autospawn_node = Node(
        package="conveyor_autospawn",
        executable="conveyor_autospawn_node",
        name="conveyor_autospawn",
        output="screen",
        parameters=[{
            "speed": LaunchConfiguration("speed"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "x_min": LaunchConfiguration("x_min"),
            "x_max": LaunchConfiguration("x_max"),
            "min_interval_s": LaunchConfiguration("min_interval_s"),
            "max_interval_s": LaunchConfiguration("max_interval_s"),
        }],
    )

    detector_node = Node(
        package="box_detector",
        executable="box_detector_node",
        name="color_detector",
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "min_fraction": LaunchConfiguration("min_fraction"),
        }],
    )

    viewer_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_view")),
    )

    return LaunchDescription([
        speed_arg,y_arg,z_arg,x_min_arg,x_max_arg,min_int_arg,max_int_arg,
        image_topic_arg, min_frac_arg,show_view_arg,
        include_world,
        autospawn_node,
        detector_node,
        viewer_node
    ])