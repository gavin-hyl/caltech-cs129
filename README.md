# Autonomous Rover Navigation in Grid Worlds

## Overview
The system built was a robot that could explore a Manhattan-style map defined by dark lines on
the ground. Abilities include undirected explore, directed explore, path-finding,
and autonomous/manual transitions between the three. The robot handles unexpected
blockages with a multi-level replan logic and is able to handle user input from both
the terminal and ROS in real time.

## Contributors (Team Skeletons)

- Gavin Hua
  - path-finding, exploring, and localization algorithms
  - motor and sensor (IR, magnetometer, and ultrasound) drivers
  - initialization/calibration and error handling
  - timer and filter functions
  - terminal UI and ROS integration
- Andrey Korolev
  - line/tunnel following algorithms
  - path-finding algorithm
  - ADC drivers
  - thread integration
- Baaqer Farhat
  - turning correction algorithm
  - calibration and tuning
