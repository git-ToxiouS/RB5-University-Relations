# RB5-University-Relations
A repository for files that are useful for the Qualcomm University relations kit that is based on the RB5 robotics board
This readme will heighlight the use of some files:

### RB5-bracket.STL
This is a 3d-model that is able to be printed so that the RB5 board can be attached to the mBot base. The bracket is attached to the mBot with spare screws from the kit but for the board itself 4-6 M2.5 screws need to be found somewhere yourself.

### pic.launch.xml
A launch file for the [gscam](https://github.com/ros-drivers/gscam) ROS2 package, specifically written for the RB5 drivers.

### photo_node.py
The ROS2 photo node made with gstreamer as described in the guide. It is dependant on the joy_node that is also present on the RB5.

### rb5_mpi_control.py
A modified version of the control script given with the [Autonomous Vehicles Laboratory](https://github.com/AutonomousVehicleLaboratory/rb5_ros2), this one is modified to have different speed settings and some other extra functions.
