# Robotics Systems Coursework

This repository contains my coursework submissions for the **Robotics Systems** module as part of my Robotics MSc program. The coursework demonstrates practical implementation of autonomous robotics systems using Arduino-based platforms.

## Project Overview

This assignment demonstrates practical implementation and programming of autonomous robotic systems across two main projects:

- **Assignment 1**: Autonomous Navigation and Object Detection
- **Assignment 2**: Autonomous Box Pushing with Angle Adjustment

## Project Structure

```
├── README.md                           # This file
├── Assignment1/
│   ├── Code/                          # Source code
│   │   ├── Assignment_1.ino           # Main program with FSM
│   │   ├── Motors.h                   # Motor control class
│   │   ├── PID.h                      # PID controller implementation
│   │   ├── LineSensors.h              # Line sensor calibration and detection
│   │   ├── Magnetometer.h             # 3-axis magnetometer interface
│   │   ├── Kinematics.h               # Robot kinematics and odometry
│   │   └── Encoders.h                 # Quadrature encoder handling
│   └── Robotics System Assignment 1.mov # Video demonstration
└── Assignment2/                       # Source code 
    ├── Baseline/                   # Basic box pushing implementation
    │   ├── Baseline.ino           # Main program
    │   ├── Motors.h               # Motor control class
    │   ├── PID.h                  # PID controller implementation
    │   ├── Encoders.h             # Quadrature encoder handling
    │   └── Kinematics.h           # Robot kinematics and odometry
    └── Improvement/                # Enhanced implementations
        ├── Improve_left/           # Left-side robot code
        │   ├── Improve_left.ino   # Left-side angle correction strategy
        │   └── [Supporting headers]
        └── Improve_right/          # Right-side robot code
            ├── Improve_right.ino  # Right-side angle correction strategy
            └── [Supporting headers]
```

## Assignment 1: Autonomous Navigation and Object Detection

**Objective:** Program an autonomous robotic system capable of navigating a bounded environment, detecting and localizing magnetic objects, and completing the following task sequence:
1. Navigate the environment until a magnetic object is detected
2. Stop, record, and store the object's position
3. Return autonomously to the starting position
4. Re-navigate to the previously detected magnetic object using the shortest path and stop at the location

### Technical Implementation
- **Sensors**: 5-line sensor array, LIS3MDL magnetometer, quadrature encoders
- **Control**: PID controllers for motor control and navigation
- **Navigation**: Line following with obstacle avoidance algorithms
- **Detection**: 3-axis magnetometer programming for object localization
- **Localization**: Dead reckoning using differential drive kinematics

### Key Features
- State machine architecture for task management
- Multi-sensor programming and calibration
- Real-time position tracking using wheel encoders
- Serial interface for debugging and data logging

## Assignment 2: Autonomous Box Pushing with Angle Adjustment

**Objective:** Program a robot system capable of autonomously pushing a box while maintaining proper orientation using bump sensors and communication-free coordination strategies.

### Technical Implementation
- **Sensors**: Dual bump sensors with time-based measurement
- **Control**: Dual PID controllers for independent wheel control
- **Strategy**: Cross-coupling control for angle adjustment
- **Programming**: Interrupt-driven control loops for real-time performance
- **Calibration**: Dynamic sensor calibration during operation

### Key Features
- Adaptive control with real-time adjustment based on sensor feedback
- Multi-robot coordination without direct communication
- Performance optimization through multiple implementation approaches
- Angle correction for maintaining proper box orientation

## Technical Skills Demonstrated

### Hardware Programming
- **Microcontroller Programming**: Arduino/AVR programming with interrupts
- **Sensor Programming**: Analog and digital sensor interfacing and calibration
- **Motor Control**: PWM-based motor control with direction control
- **Real-time Systems**: Interrupt-driven programming and timing control

### Software Engineering
- **Object-Oriented Design**: Modular class-based architecture
- **State Machine Design**: Robust finite state machine implementation
- **Control Theory**: PID controller design and tuning
- **Algorithm Implementation**: Navigation, localization, and path planning

### Robotics Concepts
- **Mobile Robot Kinematics**: Differential drive robot modeling and implementation
- **Sensor Data Processing**: Multi-sensor data integration and calibration
- **Autonomous Navigation**: Path planning and obstacle avoidance algorithms
- **Localization**: Dead reckoning and odometry implementation

## Key Results Summary

| Assignment | Algorithm | Key Achievement | Performance |
|------------|-----------|----------------|-------------|
| 1 | State Machine + PID | Autonomous navigation | 100% task completion |
| 1 | Magnetometer Detection | Object localization | Reliable detection |
| 2 | Baseline Control | Basic box pushing | Functional implementation |
| 2 | Improved Control | Angle correction | Enhanced stability |

## Learning Outcomes

This project demonstrates mastery of:

1. **Autonomous Robotics**: Programming and implementation of autonomous robotic systems
2. **Sensor Programming**: Multi-sensor systems programming with calibration and data processing
3. **Control Systems**: Real-time control implementation using PID controllers
4. **Navigation Algorithms**: Path planning, obstacle avoidance, and localization programming
5. **Embedded Programming**: Low-level programming with interrupts and timing

## Hardware Requirements

- Arduino-compatible microcontroller (3Pi robot platform)
- 5-line sensor array
- LIS3MDL 3-axis magnetometer
- Dual bump sensors
- Quadrature encoders
- Differential drive motors

## Getting Started

### Running the Code
1. **Assignment 1**: Upload `Assignment_1.ino` to Arduino
   - All header files must be in the same directory
   - Calibration routine runs automatically on startup
2. **Assignment 2**: Choose implementation version
   - `Baseline.ino`: Basic box pushing
   - `Improve_left.ino` / `Improve_right.ino`: Enhanced versions
   - All supporting headers included in each directory

### Video Demonstration
- **Assignment 1**: `Robotics System Assignment 1.mov` shows complete task execution

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

*This repository demonstrates practical robotics programming skills including sensor integration, control systems, and autonomous navigation algorithms.*