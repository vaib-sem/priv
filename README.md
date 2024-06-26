# Project Name : Morphobot
Overview

This repository contains code for controlling a rover or robot, focusing on 1. aruco marker detection 2. Autonomous movement 3. Model inference

Here are brief summaries for each of the three nodes in the project
1. controller.py - Manages the rover's actuators and controls, executing commands based on inputs from other nodes such as navigation decisions or sensor data.
2. model_inference.py - Performs inference using machine learning models, analyzing data from sensors or images to make decisions or predictions relevant to the rover's operation.
3. aruco_detection.py - Detects and processes ArUco markers in the environment, providing localization or navigation cues to the rover's control system.
