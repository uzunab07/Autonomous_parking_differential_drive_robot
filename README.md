# Autonomous Parking Differential Drive Robot

This project implements an autonomous parking solution for a differential drive robot in a structured parking-lot environment. The system is developed using ROS 2 and Python, combining path planning, control, and real-time simulation.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Technologies Used](#technologies-used)
- [Installation](#installation)
- [Usage](#usage)
- [Authors](#authors)
- [Acknowledgments](#acknowledgments)

## Overview
The project focuses on enabling a robot to autonomously navigate from a start location to a designated parking spot while avoiding obstacles. The path is calculated using the A* algorithm, and the robot follows the path using a pure pursuit control algorithm. The environment and obstacles are simulated, providing a realistic testbed for the robot's functionality.

## Features
- **Path Planning:** A* algorithm implementation for finding the shortest path.
- **Dense Path Generation:** Interpolates waypoints for smoother navigation.
- **Real-Time Control:** Pure pursuit algorithm for following waypoints.
- **Simulation:** Integrated with Gazebo for realistic robot simulation.
- **Obstacle Avoidance:** Randomly generated obstacles in the parking environment.

## Technologies Used
- **ROS 2 (Robot Operating System):** Middleware for robot development.
- **Python 3:** Core programming language.
- **Gazebo:** Simulator for robot and environment modeling.
- **RViz 2:** Visualization of the robot's state and path.

## Installation
1. Clone this repository:
    ```bash
    mkdir autonomous-parking
     cd autonomous-parking
    git clone https://github.com/your-repo/autonomous-parking.git
   
    ```

2. Install required dependencies:
    *(Ensure you have ROS 2 and its dependencies installed before proceeding.)*
     *Ensure that you have turtlebot or limo packages installed*

4. Build the ROS 2 workspace:
    ```bash
    colcon build
    source install/setup.bash
    ```
## Usage
1. Launch the robot and simulation environment:
    ```bash
    ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
    ```

2. Run the autonomous parking node:
    ```bash
    ros2 run autonomous_parking my_model "(x1, y1)" "(x2, y2)" ...
    ```


## Authors
- **Bryan Faryadi**
- **David Oh**
- **Khaled Mohamed Ali**

## Acknowledgments
- **Instructor:** Wenhao Luo, for guidance throughout the project.
- **UNC Charlotte (UNCC):** For providing the learning environment and resources.

