# dxl_interface
Dynemixel SDK interface

## Dependencies
This package depends on DynamixelSDK package, which must be installed
before using this package

## Start Up
This package contain demo launch file, node and config files.
To use this package in your code:

1. add dxl_interface to your package.xml and Cmakelist files

2. include dxl_interface.h and dxl_motor_builder.h

3. use example_node.cpp file as an example 

4. create launch and dxl_joints_config in your package based on the original files from dxl_interface package. Edit those files to suit your use case.

If you wish to control the motors directly, you can invoke getMotor() or setMotorPosition() / setMotorVelocity() to control motors. If you wish to control motors with ROS controller, use registerHandles() to register joints to ROS controller manager
edit config files according to your arm configuration.
