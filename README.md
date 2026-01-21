# conveyor-gate
A ROS2 project creating a cube sorting conveyor gate
## Modules
### conveyor-model
Forked module from IFRA-Cranfield Gazebo-ROS2 Conveyor Belt Plugin. With the addition of a swing arm robot and camera model, PID joint trajectory controler for the swing arm model, updated world file, and launch file.
IFRA-Cranfield (2023) Gazebo-ROS2 Conveyor Belt Plugin. URL: https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt. 
### box_detector
Box colour detector node using colour masks. Publishes detected colour to `detected_box_colour` topic as well as debug image viewer with detected colour drawn on image.
### conveyor_autospawn
Loads and spawns boxes with "random" colour (Red/Blue), position and time. Waits for convyor model to load and sets conveyor belt speed
### swing_arm_controller_node
Moves swing arm model to correct position based on output from the box detector.
### conveyor_bringup
Launch package that brings up the complete simulation. Launches the Gazebo world, box autospawner, colour detector, swing arm robot with its controllers,optional debug image viewers, and launch arguments for every module.
## Bring up
`ros2 launch conveyor_bringup sim_autospawn_with_detector.launch.py`
with debug view
`ros2 launch conveyor_bringup sim_autospawn_with_detector.launch.py show_debug:=true`
