# JD Edu ROS2 Study Example code 
## Introduction 
This repository contains a couple of ROS2 packages. Packages are written by mainly Python.
ROS2 is humble and Ubuntu 22.04 LTS. 
## ros_topic
This package shows ROS2 topic function. It is written by C++. 

## ros_topic_py
This package shows ROS2 topic function. It is written by Python. 

## ros_serial_py
Using this package, you can wirte ROS2 serial control node. This package includes arduino uno serial code. 

## ros_cv_py
Using this package, you can write ROS2 OpenCV control node. You can make camera publisher node and subscariber node. 

## motor_control
This package shows ROS2 service client function. It is written by Python. Right now this code has bug and NOT working 

## jdamr_ros_01
This package show very basic ROS based wheel robot. "jdamr_ros_xx" is step-by-step tutorials.
In jdamr_ros_01, you can learn to communicate with arduino through serial. You can send integer value that issimulated encoder value 
You can write code to control robot encoder. 

## jdamr_ros_02
This package show next level ROS2 ode code from jd_amr_ros_01.
In this node, you can learn bi-directonal serial communication through serial.
From ROS2 node, you can send motor control command using teleop. 
