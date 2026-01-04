from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Autospawn args
    speed_arg        = DeclareLaunchArgument("speed", default_value="50")
    y_arg            = DeclareLaunchArgument("y", default_value="-0.5")
    z_arg            = DeclareLaunchArgument("z", default_value="0.76")
    x_min_arg        = DeclareLaunchArgument("x_min", default_value="-0.05")
    x_max_arg        = DeclareLaunchArgument("x_max", default_value="0.05")
    min_int_arg      = DeclareLaunchArgument("min_interval_s", default_value="2.0")
    max_int_arg      = DeclareLaunchArgument("max_interval_s", default_value="6.0")

    # Swing arm controller args

    pid_p_arg = DeclareLaunchArgument('pid_p', default_value='100.0')
    pid_i_arg = DeclareLaunchArgument('pid_i', default_value='0.0')
    pid_d_arg = DeclareLaunchArgument('pid_d', default_value='50.0')

    swing_angle_arg = DeclareLaunchArgument('swing_angle', default_value='0.523599') # 30 degrees
    debounce_arg = DeclareLaunchArgument('debounce_duration', default_value='3.0')
    move_duration_arg = DeclareLaunchArgument('move_duration', default_value='0.5')

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

    xacro_file = os.path.join(conv_share, "models", "swing_arm2", "swing_arm2.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml() 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'swing_arm',
            '-x', '0.0',
            '-y', '0.7',
            '-z', '0.125',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '3.14'
        ]
    )

    swing_arm_node = Node(
        package="swing_arm_controller_node",
        executable="swing_arm_controller_node",
        name="swing_arm_controller",
        output="screen",
        parameters=[{
            'swing_angle': LaunchConfiguration('swing_angle'),
            'debounce_duration': LaunchConfiguration('debounce_duration'),
            'move_duration': LaunchConfiguration('move_duration'),
        }]
    )

    spawn_joint_state_broadcaster = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    parameters=[{
        'arm_controller.gains.boom_link_JOINT_0.p': LaunchConfiguration('pid_p'),
        'arm_controller.gains.boom_link_JOINT_0.i': LaunchConfiguration('pid_i'),
        'arm_controller.gains.boom_link_JOINT_0.d': LaunchConfiguration('pid_d'),
    }]
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller']
        )


    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_joint_state_broadcaster, spawn_arm_controller]
        )
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
        pid_p_arg,pid_i_arg,pid_d_arg,swing_angle_arg,debounce_arg,move_duration_arg,
        image_topic_arg, min_frac_arg,show_view_arg,
        include_world,
        autospawn_node,
        detector_node,
        spawn_robot,
        robot_state_publisher,
        swing_arm_node,
        viewer_node,
        delay_controllers,
    ])