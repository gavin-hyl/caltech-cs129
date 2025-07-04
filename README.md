# Autonomous Rover Navigation in Grid Worlds

## Overview
This system implements a sophisticated **autonomous navigation framework** for robotic rovers operating in **Manhattan-style grid environments**. The architecture integrates **multi-sensor fusion**, **dynamic path planning**, and **real-time obstacle avoidance** to enable robust exploration and navigation of unknown grid-based worlds defined by dark line boundaries.

Key capabilities include **undirected exploration**, **goal-directed navigation**, **dynamic replanning** with multi-level fallback strategies, and **seamless autonomous/manual mode transitions**. The system demonstrates exceptional resilience to environmental uncertainties, sensor noise, and unexpected obstacles through intelligent **graph-based mapping** and **adaptive decision-making algorithms**.

The implementation features comprehensive **ROS integration** for remote operation, **real-time sensor fusion** from IR, ultrasound, and magnetometer sensors, and **thread-safe multi-subsystem coordination** for robust autonomous operation.

## Architecture
The system consists of four primary subsystems operating in coordinated threads:

1. **Navigator Thread**: Core navigation logic with decision-making and path execution
2. **Graph Management Thread**: Real-time mapping, pathfinding, and exploration planning  
3. **Sensor Fusion Thread**: Ultrasound proximity detection and obstacle avoidance
4. **ROS Communication Thread**: Remote command interface and pose publishing

## Core Technical Components

### Intelligent Navigation System
- **Multi-mode Operation**: Seamless transitions between exploration, goal-seeking, and manual control
- **Line Following**: Robust detection and tracking of grid boundaries with intersection recognition
- **Obstacle Handling**: Dynamic obstacle detection with multi-level replanning strategies
- **Localization**: Integrated odometry with magnetometer-based heading correction

### Graph-Based Mapping & Planning
- **Dynamic Graph Construction**: Real-time mapping of grid topology with street status tracking
- **Dijkstra-Based Pathfinding**: Optimal path generation with cost-based route selection
- **Exploration Strategy**: Intelligent frontier detection prioritizing unexplored vs. unknown regions
- **Blockage Management**: Adaptive handling of temporary and permanent obstacles

### Multi-Sensor Fusion
- **Proximity Detection**: Triple ultrasound sensor array for obstacle avoidance
- **Line Detection**: IR sensor array for boundary tracking and intersection recognition
- **Heading Correction**: Magnetometer integration for drift compensation
- **Sensor Validation**: Multi-sensor correlation for robust environmental perception

### Thread-Safe Communication
- **SharedData Architecture**: Lock-based synchronization for multi-threaded operation
- **Real-time Decision Making**: Pause/step execution control with responsive user interaction
- **Status Management**: Comprehensive state tracking across all subsystems

## Installation

### Prerequisites
- Python 3.8+
- ROS 2 (Humble or later)
- pigpio library for GPIO control
- NumPy, Matplotlib
- Hardware: IR sensor array, ultrasound sensors, magnetometer

### Hardware Setup
The system requires:
- **IR Sensor Array**: For line detection and intersection recognition
- **Ultrasound Sensors**: Triple-sensor configuration for obstacle detection
- **Magnetometer**: For heading correction and drift compensation
- **Motor Controllers**: For differential drive control

### Software Installation
```bash
# Install Python dependencies
pip install pigpio numpy matplotlib

# Install ROS 2 dependencies
sudo apt install ros-humble-geometry-msgs ros-humble-std-msgs

# Clone and setup the project
git clone <repository-url> rover-grid-nav
cd rover-grid-nav
```

## Usage

### System Startup
```bash
# Start the main navigation system
python3 UI.py
```

The system will prompt for initial pose configuration and then enter the interactive command interface.

### Command Interface
The system supports both terminal and ROS-based control:

#### Terminal Commands
- `explore`: Begin autonomous exploration of unknown areas
- `goal`: Set navigation target (prompts for coordinates)
- `pause`/`resume`: Pause/resume autonomous operation
- `step`: Execute single navigation step
- `left`/`right`/`forward`: Manual movement commands
- `save`/`load`: Graph persistence operations
- `pose`: Update robot localization
- `show`: Display current map visualization
- `quit`: Shutdown system

#### ROS Interface
```bash
# Send goal command
ros2 topic pub /HOSTNAME/goal geometry_msgs/Point "x: 5.0, y: 3.0, z: 0.0"

# Send explore command  
ros2 topic pub /HOSTNAME/explore std_msgs/Empty

# Monitor robot pose
ros2 topic echo /HOSTNAME/pose
```

## ROS 2 Topic Structure

### Published Topics
- `/HOSTNAME/pose`: Current robot pose with position and orientation (`geometry_msgs/Pose`)

### Subscribed Topics
- `/HOSTNAME/goal`: Goal coordinates for navigation (`geometry_msgs/Point`)
- `/HOSTNAME/explore`: Exploration mode activation (`std_msgs/Empty`)

### Internal Communication
The system uses thread-safe SharedData objects for internal coordination:
- **Navigation State**: Current mode, goal, and execution status
- **Graph Data**: Dynamic map with street statuses and pathfinding results
- **Sensor Data**: Real-time proximity and environmental measurements
- **Control Signals**: Pause/step execution and manual override commands

## Technical Implementation Details

### Navigation Algorithms

#### Line Following
- **Intersection Detection**: Multi-sensor correlation for T-junctions and crossroads
- **Tunnel Recognition**: Specialized handling for covered pathway segments
- **End-of-Line Detection**: Automatic U-turn execution for dead ends
- **Drift Correction**: Magnetometer-based heading validation and correction

#### Path Planning
- **Dijkstra Implementation**: Optimal pathfinding with dynamic cost updates
- **Multi-level Replanning**: Hierarchical fallback strategies for obstacle handling
- **Exploration Priority**: Intelligent frontier selection balancing distance and information gain
- **Blockage Adaptation**: Dynamic graph updates for temporary and permanent obstacles

#### Decision Making
- **State Machine**: Robust mode transitions with error handling
- **Priority System**: Goal-seeking override with exploration fallback
- **Manual Override**: Immediate response to user commands with safety checks
- **Error Recovery**: Automatic recovery from sensor failures and navigation errors

### Sensor Integration

#### Proximity Detection
- **Triple Sensor Array**: Redundant ultrasound measurements for reliability
- **Adaptive Thresholding**: Dynamic obstacle detection based on environmental conditions
- **Filtering**: Noise reduction and outlier rejection for stable detection
- **Collision Avoidance**: Real-time obstacle response with replanning

#### Environmental Perception
- **IR Array Processing**: Multi-sensor line detection with intersection classification
- **Magnetometer Fusion**: Heading correction with drift compensation
- **Sensor Validation**: Cross-sensor correlation for robust perception
- **Calibration**: Automatic sensor calibration and drift compensation

### System Robustness

#### Error Handling
- **Sensor Failure Recovery**: Graceful degradation with reduced functionality
- **Navigation Error Detection**: Automatic detection of lost robot status
- **Replanning Strategies**: Multi-level fallback for blocked paths
- **Safe Mode Operation**: Emergency stop and manual override capabilities

#### Performance Optimization
- **Thread Synchronization**: Efficient lock-based coordination
- **Memory Management**: Optimized graph storage and retrieval
- **Real-time Operation**: Responsive control with minimal latency
- **Resource Monitoring**: CPU and memory usage optimization

## Configuration Parameters

### Navigation Parameters
- **Line Following Sensitivity**: IR sensor threshold adjustment
- **Turn Validation**: Magnetometer-based turn completion verification
- **Obstacle Threshold**: Ultrasound distance thresholds (default: 0.6m)
- **Replanning Strategy**: Multi-level fallback configuration

### Sensor Configuration
- **Ultrasound Range**: Min/max detection distances
- **IR Sensitivity**: Line detection threshold parameters
- **Magnetometer Calibration**: Heading correction parameters
- **Sensor Fusion Weights**: Multi-sensor integration coefficients

### System Behavior
- **Exploration Strategy**: Frontier selection algorithm parameters
- **Pathfinding Costs**: Distance and information gain weighting
- **Error Recovery**: Timeout and retry configuration
- **Communication Timing**: Thread synchronization parameters

## Advanced Features

### Intelligent Exploration
- **Frontier Detection**: Automated identification of unexplored boundaries
- **Priority-Based Selection**: Distance-weighted exploration target selection
- **Adaptive Strategy**: Dynamic exploration behavior based on map completeness
- **Blockage Handling**: Intelligent response to temporary and permanent obstacles

### Robust Navigation
- **Multi-Sensor Fusion**: Redundant sensor integration for reliability
- **Error Recovery**: Automatic recovery from navigation failures
- **Dynamic Replanning**: Real-time path adjustment for changing conditions
- **Safe Operation**: Emergency stop and manual override capabilities

### Remote Operation
- **ROS Integration**: Full remote control and monitoring capabilities
- **Real-time Telemetry**: Continuous pose and status broadcasting
- **Command Interface**: Comprehensive remote command execution
- **Status Monitoring**: Real-time system health and performance metrics

## System Validation

The system has been extensively tested in various grid environments demonstrating:
- **Navigation Accuracy**: Consistent path following with minimal drift
- **Obstacle Handling**: Robust response to unexpected blockages
- **Exploration Efficiency**: Systematic coverage of unknown areas
- **System Reliability**: Stable operation under various environmental conditions
