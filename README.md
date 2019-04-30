# AA241x Student Commander Example #

This ROS package is a skeleton for the teams of Spring 2019's AA241x course.  The purpose of this package is mission handling / command and control of the drone through the Pixhawk 4.  This package has been created separately from the image handling nodes as the dependencies for the nodes related to the camera and imagine are quite different than the dependencies for the mission / command and control related nodes.  Furthermore, the separation allows teams to test the mission / command and control nodes on the desktops running Gazebo (i.e. not the Raspberry Pi 3B+) without any compilation problems that would occur due to the fact that the imaging handling nodes are very specific to the Raspberry Pi 3B+ hardware (i.e. the PiCam v2) and these nodes can be more generalized to any hardware interacting with the PX4 code and the world environment.

This skeleton package currently contains the following node:

 - [`control_node`](#control-node): a skeleton node to help get you started with the command and control of the Pixhawk 4 onboard the drone through the [MavROS](http://wiki.ros.org/mavros) interface.  Currently the node executes a takeoff to a given altitude and once that altitude is reached, commands a landing using position commands.


## Getting Started ##

The follow sections will help you get started with this ROS package.

### Getting the Code ###

It is recommended that you first [fork](https://help.github.com/en/articles/fork-a-repo) the repository to allow you to make changes and build off the code as desired in your own fork of the code.

Once forked, you will be able to clone your repository on to your team's Raspberry Pi 3B+.  This is a ROS package that has been designed with the `catkin_ws` that is used with ROS Melodic, so make sure to clone this repository into the `catkin_ws/src` directory, where ROS packages live.

For example:

```sh
cd ~/catkin_ws/src
git clone https://github.com/<your-github-handle>/aa241x_commander.git
```

Once you have it cloned, you can build the code using `catkin_make`:

```sh
cd ~/catkin_ws/
catkin_make
```

### Running the Code ###

Each of the nodes of this package can be run using the basic [`rosrun`](http://wiki.ros.org/rosbash#rosrun) framework or can be run using the example launch file using `roslaunch`.  For a tutorial on `roslaunch` check out either [this resource](http://www.clearpathrobotics.com/assets/guides/ros/Launch%20Files.html) or [this resource](http://wiki.ros.org/roslaunch#Tutorials).  The example launch file (`controller_only.launch`) will start the connection to the gazebo simulation (using the help of a launch file contained in [`aa241x_mission`](https://github.com/aa241x/aa241x_mission)) and the control node.

To run using the launch file, you first need to make sure that you have the `aa241x_mission` package.  You will most likely not be modifying the `aa241x_mission` package, so feel free to just [clone the package](https://github.com/aa241x/aa241x_mission), but if you are interested in making your own modification, you can also fork the package.

Once you have the `aa241x_mission` package, you can launch the `controller_only.launch` file:

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_commander controller_only.launch
```

**Note:** for more details on the connection to the gazebo simulation using mavros, see the `aa241x_mission` [documentation](https://github.com/aa241x/aa241x_mission).

#### Additional Launch File ####

This skeleton node also contains a launch file to demonstrate the use of [`rosbag`](http://wiki.ros.org/rosbag) to be able to log data published to topics.  This is a very helpful tool to be able to log all the data and enable replaying data in ROS after a flight.

**Note:** For the logging launch file to work, you will need to make sure to have the following directory `~/rosbags/` (if needed `mkdir ~/rosbags`) as that is where the logs will be saved.  More details on `rosbag` (logging, playback, etc), check out the [general documentation for the `aa241x_mission` node](https://github.com/aa241x/aa241x_mission).

### Dependencies ###

This code has the following dependencies:

 - [MavROS](http://wiki.ros.org/mavros) - this is a ROS package that handles the communication with PX4 using the Mavlink protocol.  It handles the communication and exposes the information by publishing the information using the ROS topic framework.


## Nodes ##

Here is a more detailed description of the control node.

### Control Node ###

The control node serves both as a skeleton for the development of team's controllers for the Spring 2019's AA241x mission and as an example for controlling a drone through using the MavROS framework for sending control commands to PX4.  The example controller sends position commands to the PX4 to execute a takeoff and once the drone reaches the specified altitude commands the drone to land, again through a position control command.  To achieve this, the node subscribes to several pieces of critical information and publishes commands to a topic that the MavROS node subscribes to (see below for details on the subscriptions and publications).

#### Subscribes To ####

The control node subscribes to the following information:

 - `mavros/state` - this topic, of type [`mavros_msgs::State`](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/State.html), contains information on the state of the PX4 code.  The node uses 2 pieces of information from the state:
     + `connected` - a boolean for whether or not the PX4 has completed the bootup process and has connected to the offboard computer (e.g. your Raspberry Pi 3B+).
     + `mode` - a string stating the current control mode the PX4 is set to (e.g. "MANUAL" or "OFFBOARD")

 - `/mavros/local_position/pose` - this topic, of type [`geometry_msgs::PoseStamped`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html), contains the local position (ENU) (defined with the origin as the take-off position of the drone) and the orientation of the drone computed by PX4.

 - `measurement` - this topic, of type [`aa241x_mission::SensorMeasurement`](https://github.com/aa241x/aa241x_mission/blob/master/msg/SensorMeasurement.msg), contains the sensor information for the AA241x mission.  For more details on the sensor measurement, check out the [documentation for the `aa241x_mission` node](https://github.com/aa241x/aa241x_mission).

 - `mission_state` - this topic, of type [`aa241x_mission::MissionState`](https://github.com/aa241x/aa241x_mission/blob/master/msg/MissionState.msg), contains general state information for the mission, and the offset needed to go from the ENU frame as computed by PX4 (with origin as the take-off position) to the Lake Lag ENU frame.  For more details on the Lake Lag frame and how to use the offset information, check out the [documentation for the `aa241x_mission` node](https://github.com/aa241x/aa241x_mission).

**Note:** You will most likely find that you need to subscribe to additional information, so check out the [full MavROS documentation](http://wiki.ros.org/mavros) for additional topics that are published.  To help see what topics and data are publishes, check out [`rostopic`](http://wiki.ros.org/rostopic), which enables viewing the published topic names and data (among other things).

#### Publishes ####

 - `mavros/setpoint_raw/local` - this topic, of type [`mavros_msgs::PositionTarget`](http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html), provides the control information to send to PX4 for controlling the drone.  The command is made up of several key elements:
     + `type_mask` - a bitfield that tells PX4 which fields to **ignore**.  For the most part the bitfield can be built using the constants defined in the topic type (the only notable exception is an altitude hold velocity control (specify Pz, Vx, Vy) which has a mask value of `2499`).
     + the data fields: `position`, `velocity`, `acceleration_or_force`, `yaw`, and `yaw_rate`.  These fields contain the respective information for the command to send to PX4.  **Note:** if you set data to the `velocity` fields, but the `type_mask` bitfield specifies to ignore the `velocity` fields, PX4 will ignore the fields, regardless of the data contained within the field.

## MavROS Highlights ##

, however, here is set of important highlights to the documentation for elements used in the control node.






