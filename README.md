# ros-roboterpraktikum

Piep. Kaputt.

## Prerequisites

First, you will need [ROS](https://www.ros.org/) on your operating system.

The required packages are:

- ros-melodic-desktop
- ros-melodic-navigation

If you are running Ubuntu, please follow the installation on the ROS Wiki:
https://wiki.ros.org/melodic/Installation/Ubuntu

## Installation

Place the workspace folder (ros-roboterpraktikum) at a place of your desire (in the following, referenced as $HOME). Run `catkin_make` in the folder `ros-roboterpraktikum` to build the project. To be able to use the project, type `source $HOME/ros-roboterpraktikum/devel/setup.bash`. You might want to add this line to your `.bashrc`.

## Usage

For all examples, it is assumed that the robot is connected to your notebook via LAN and USB, the `/dev/volksbot` symlink exists and a static IP address is set (according to the instructions in the other README).

### Basic driving

Run `roslaunch volksbot robot.launch` in one shell and `roslaunch control js.launch` in another. Use the sticks to control the wheel speeds.

### Generate a new map

Place the robot at the start position. Run the commands from basic driving to get maneuverability. Start a gmapping node with `roslaunch volksbot map-recorder.launch`. You might want to see the map in creation, therefore use `roslaunch volksbot map-viewer.launch`.  Drive slowly and near the walls to get an accurate map. To save the map, use the `map_saver` (`rosrun map_server map_saver`). To use the map, place it into `$HOME/ros-roboterpraktikum/src/volksbot/ressources/maps/`. The default map is called `map.pgm` with the config `map.yaml`. You can specify this file in `$HOME/ros-roboterpraktikum/src/volksbot/launch/map-server.launch`.

### Generate a new path

Place the robot at the start position. Start the basic driving commands. Start `roslaunch volksbot map-server.launch`. Start `roslaunch volksbot amcl.launch`. Make sure that `amcl` is set as `/path/source` in `$HOME/ros-roboterpraktikum/src/control/config/gio_control.yaml`. Drive to the start point of your path. Start `roslaunch volksbot path-recorder.launch` to record the path. Drive slowly. Stop `path-recorder` to stop the recording. The new path is placced into `/tmp/recording*.dat`. You need to copy this file into `$HOME/ros-roboterpraktikum/src/control/ressources/paths/` to be able to use it with the next Giovanni controller.

### Use odometry for the Giovanni controller

Run the commands from basic driving. Make sure that `odom_pose` is set as `/path/source` in `$HOME/ros-roboterpraktikum/src/control/config/gio_control.yaml`. Run `roslaunch control gio.launch file:=$YOURPATH.DAT`, `$YOURPATH.DAT` is a path file from the `$HOME/ros-roboterpraktikum/src/control/ressources/paths` directory. Be careful, the robot will start instantanously.

### Use amcl for the Giovanni controller

Run the commands from basic driving. Start the map server (`roslaunch volksbot map-server.launch`) and then amcl (`roslaunch volksbot amcl.launch`). Set `/path/source` to `amcl` in `$HOME/ros-roboterpraktikum/src/control/config/gio_control.yaml`. Run `roslaunch control gio.launch file:=$YOURPATH.DAT` to follow the path.

## Contributing

Feel free to use and fix and add stuff.
