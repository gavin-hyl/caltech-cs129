# Caltech CS/EE/ME 129 - Experimental Robotics

## Description

### Course Description (Caltech 2023-2024 [Catalog](https://www.cms.caltech.edu/academics/courses/mecsee-129))

This course covers the foundations of experimental realization on robotic systems.
This includes software infrastructure to operate physical hardware, integrate
various sensor modalities, and create robust autonomous behaviors.
Using the Python programming language, assignments will explore techniques from
simple polling to interrupt driven and multi-threaded architectures, ultimately
utilizing the Robotic Operating System (ROS). Developments will be integrated on
mobile robotic systems and demonstrated in the context of class projects.

### Project Description

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
