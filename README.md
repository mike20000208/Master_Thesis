# ROS packages for the master project - "Navigation in grass-areas for autonomous farm robot"

The project aims to make the four-wheel robot autonomously manoeuvre in a fenced farm full of grass and other obstacles, such as livestock and branches. 

## Table of Contents

- ROS node for simulation in C++ - cpp_capra_nodes
- Main package - cpp_main
- ROS node as a transfer station in Python - py_odo

## cpp_capra_nodes

A package used as a simulation of Capra nodes. It will send some GPS and odometry data to test if the communication system works so that I won't need to actually turn on the robot and be able to develop it from home. 

## cpp_main

A package that contains all the main programmes, which carry the map generation and path-finding tasks. 

## py_odo

A package that transfers the odometry data from the MQTT client to other ROS nodes and publishes it. It maintains the consistency of the programming language used. 