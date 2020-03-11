# URS100

 This repository contains a complete implementation of the [Newport URS100](https://www.newport.com/p/URS100BCC) rotary stage for use with the Robot Operating System (ROS).  
 The higher-level package `urs100` is a metapackage, which contains the following packages:

| Package                           | Description                                                                                           |
| --------------------------------- |:-----------------------------------------------------------------------------------------------------:|
| urs100_driver                     | Direct interface of the rotation axis, which uses the serial commands from the official documentation |
| urs100_hardware_interface         | Controller following the convention of [ros_control](http://wiki.ros.org/ros_control)                 |
| urs100_support                    | Support package, follwing the [ROS-I standards](https://rosindustrial.org)                            |