# Control Guide

## Overview 

This package contains a collection of control inputs for mobile robots. There are three choices available: A Giovanni controller following paths, a keyboard controller and a joystick controller.

## Note

The package `control` is designed to be used in combination with the `volksbot` package. You need to be able to execute `volksbot` programs to use this package.

Some parts of this project might not fully work, e. g. the pose reset part for the included Giovanni Controller.

This work builds upon previous work, notable Dorit Borrmann (borrmann@informatik.uni-wuerzburg.de). 

## Communication

### Topics

#### Subscribes to
 *  pose topic (`odom` or `amcl_pose`)

#### Publishes to
 *  velocity topic (`Vel`)

### Services
 *  velocity service (`Controls`) to be called by joystick and keyboard
 *  stop gio service (`stop_input_gio`) to be called and serviced

## Configuration

Right now, there are following config files:
 *  `config/gio_control.yaml` configuring in/output of the Giovanni controller
 *  `config/parameters.yaml` to configure general settings
 *  `config/rosconsole.conf` to configure the verbosity level in the console window

### Parameters to configure

 *  `topic/source` topic to subscribe for pose messages
 *  `control/looprate` looprate of the controller
 *  `robot/u_max` maximal wheel velocity for the Giovanni Controller in [cm/s]
 *  `robot/axis_length` wheel-base distance of the robot in [mm]
 *  `path/export_dir` where to write logfiles, csv files etc for the Giovanni controller
 *  `path/joystick` path to the joystick device
 *  `control/joystick/*` joystick type to use
 *  `frame/world` name of the world frame (tf)
 *  `frame/dest` name of the start frame for the Giovanni Controller
 *  `service/reset` name of the pose reset service
 *  `topic/velocity` name of the velocity topic
 *  `service/stop` name of the stop gio service
 *  `path/datfile` path input for the Giovanni Controller (absolute)
 *  `service/velocity` name of the velocity service to use

## Launchfiles

Right now, each node has it's own launch file. One might select the appropriate one.

 *  `gio.launch` to launch the Giovanni Controller, needs the argument `file` to be set while `file` is the path to use (relative to `ressources/paths/`)
 *  `js.launch` to launch the joystick controller
 *  `kb.launch` to launch the keyboard controller
