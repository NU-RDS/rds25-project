# RDS 2025 Project - Robotic Hand Control System

## Project Overview

The RDS 2025 project is a sophisticated robotic hand control system featuring three integrated subsystems: a 2-DOF wrist, a 4-DOF dexterous finger, and a 1-DOF power finger. The system employs Series Elastic Actuators (SEAs) for compliant control, tendon-driven mechanisms for compact design, and distributed control architecture across multiple microcontrollers.

### Key Features
- **Wrist Control**: 2-DOF (pitch and yaw) 
- **Dexterous Finger**: 4-DOF (PIP, DIP, MCP, Splay) with coupled tendon actuation between PIP and MCP
- **Power Finger**: 1-DOF grasp mechanism with high force capability
- **Force Control**: Series Elastic Actuators with real-time force feedback
- **Distributed Architecture**: Multi-MCU system with CAN bus communication

## System Architecture

### Workflow 

![alt text](figures-videos/image.png)

### Repo Structure
```
High-Level Controller (Teensy 4.1)
├── State Management & Coordination
├── Joint-Level Control (Position/Force)
├── Tendon Kinematics & Coupling
└── CAN Communication Hub

Low-Level Controllers (Teensy 4.1)
├── SEA Force Control
├── Motor Current Control
├── Encoder Feedback
└── Load Cell Integration

Palm Controller (Teensy 4.0)
├── Multi-Encoder Reading (8 channels)
├── SPI Communication
└── Joint Angle Feedback
```

### System Overview
-  **User Interface**: handling user input (PID gains, reference type, plotter & logger) for control and testing 
-  **High-level MCU**: responsible for parsing commands, kinematics calculation, and position PID control; will send joint torque commands to **low-level MCU**
-  **Low-level MCU**: responsible for inner force control loop with PID and tendon force calculation with SEA modules; will drive the motor with the motor driver board (in this case ODrive Pro) by sending current/torque command
-  **Palm Board/Sensor Board**: gathering joint sensor information to give it to **high-level MCU** for kinematics calculation
-  **Communication Library**: handling CAN bus communication between all the MCUs, designed for our original architecture with high-level and low-level controllers, but can be used for any other architecture as well. Stored in separate repository: [rds25-comms](https://github.com/NU-RDS/rds25-comms) to use across the `High-Level`, `Low-Level`, and `Palm` controllers

![alt text](figures-videos/image8.png)

### Mathematical Foundation

#### Tendon Kinematics
The system uses a Jacobian-based approach for tendon-to-joint mapping, with each motor's velocity mapped to its respective actuating joint's velocity through a tendon routing matrix, an example for which is shown in dex finger below:

**Dexterous Finger Jacobian:**
```
J_dex = [ 1.0  -2.0  -1.2  -1.2]
        [-1.0   0.0   0.0   0.0]
        [-1.0   2.0   1.2   1.2]
        [ 1.0   2.52  1.2   1.2]
```

More information on wrist coupling and power finger can be found in the Mechanical documentation linked below. 

**Motor Torque Calculation:**
```
τ_motor = (1/R_motor) × (J × τ_joint + τ_null)
```

Where:
- `R_motor = 5.0 mm` (motor pulley radius)
- `τ_null` provides null-space control for tendon pretension

#### Series Elastic Actuator (SEA) Calibration
Force-deflection relationships were experimentally determined for each SEA:

**Linear SEAs:**
```
F_SEA2 = 0.206 × θ
```

**Nonlinear SEAs:**
```
F_SEA3 = 0.972 × θ - 0.00432 × θ²
F_SEA4 = 0.829 × θ - 0.00714 × θ²
```

Where:
- F is the force in Newtons (N)
- θ is the angular deflection in degree (deg)

The spring characterization for all the SEAs can be found in the references below.

## Hardware Integration

### Sensor Systems
- **Encoders**: AS5047P magnetic encoders (14-bit resolution)
- **Force Feedback**: SEA measurements taken through encoder ans spring calibrated through NAU7802 load cells
- **Motor Control**: ODrive motor controllers via CAN bus

## Control Implementation

### Position Control
The system implements cascaded PD control with anti-windup:

![alt text](figures-videos/image1.png)

### Force Control
SEA-based force control with PD gains:

![alt text](figures-videos/image2.png)

## Communication Protocol

### Serial Command Interface
The system accepts commands in the format `ID:value1 value2 ...`:

```
Command Examples:
10:30.0 15.0 45.0 30.0 15.0 0.0 60.0  // Set all joint positions
20                                      // Emergency stop
30                                      // Get system status
```

### CAN Bus Communication
The system uses CAN bus for inter-controller communication:
- **Baud Rate**: 250 kbps
- **High-Level**: Joint coordination and state management
- **Low-Level**: Motor control and sensor feedback
- **Palm**: Multi-encoder data acquisition

To manage this communication, we use a combination of the `ODrive` library for Teensy and our custom `rds25-comms` library, which provides a simple interface for sending and receiving commands across the CAN bus.

For speaking to motor controllers, we use the `ODrive` library, which provides a high-level interface for controlling motors via CAN bus. The library allows us to set motor parameters, read encoder values, and control motor currents.

For communication between the various other controllers, we use our custom `rds25-comms` library. This library provides a simple interface for sending and receiving commands across the CAN bus, managing heartbeats, and handling errors. It allows us to easily send commands to the high-level controller, low-level controllers, and palm controller, ensuring that all components of the system can communicate effectively.

`rds25-comms` documentation and reflections can be found in the [rds25-comms repository](https://github.com/NU-RDS/rds25-comms).

## GUI and Monitoring

### Python Control Interface
A comprehensive Tkinter-based GUI provides:
- Real-time joint position control
- Force/position data recording
- System monitoring and diagnostics
- Preset hand configurations

![](figures-videos/image7.png)

### Data Visualization
```python
# Real-time pitch/yaw recording with statistical analysis
def plot_recorded_data(self):
    pitch_error = np.array(pitch_desired) - np.array(pitch_actual)
    yaw_error = np.array(yaw_desired) - np.array(yaw_actual)
    
    rms_error_pitch = np.sqrt(np.mean(pitch_error**2))
    rms_error_yaw = np.sqrt(np.mean(yaw_error**2))
```

## Experimental Results

### SEA Force Characterization
Each SEA was individually calibrated to determine force-deflection relationships:
- **Linear SEAs**: Simple proportional relationship (SEA_2, SEA_5, SEA_6)

![alt text](figures-videos/image3.png)

- **Nonlinear SEAs**: Quadratic compensation required (SEA_3, SEA_4, SEA_7, SEA_8)

![alt text](figures-videos/image4.png)

Indexing of SEAs in comparison to mechanical convention can be found here: [SEA Indexing](https://docs.google.com/spreadsheets/d/170EN8GTMZlCyYZjw70axoC4KJDlu1CnwCoCHKkFP4lc/edit?gid=0#gid=0)

### Force Control Validation

![alt text](figures-videos/image5.png)

Load cell feedback demonstrates:
- **Force accuracy**: ±0.1N at 5N maximum
- **Response time**: <100ms for force steps
- **Stability**: No oscillations observed in normal operation

## System Performance

### Strengths
1. **Modular Architecture**: Clean separation of concerns across subsystems
2. **Real-time Performance**: 100Hz control loop with deterministic timing
3. **Force Feedback**: Accurate SEA-based force control enables compliant manipulation
4. **Comprehensive Monitoring**: Extensive logging and visualization capabilities
5. **Robust Communication**: CAN bus provides reliable inter-controller communication

### Areas for Improvement
1. **Tendon Coupling**: Complex interdependencies require careful tuning
2. **Calibration Complexity**: Each SEA requires individual characterization
3. **Range Limitations**: Some joints have restricted ROM due to mechanical constraints
4. **Force Limits**: Current SEA design limits maximum force output
5. **Hardware Configuration**: Better architecture design to account for changes in hardware

### Future Enhancements
1. **Feedforward Control**: Using feedforward + PD control to enhance control of large subsystems
2.  **Gravity Compensation**: Working with mechanical to get a model of the gravity distribution and integrate it into the control loop to improve system stability
3.  **Zero Impedence Control**: Consider adding zero impedance control mode, which makes it easier to set up based on certain configuration
4. **Motor Drivers with SimpleFOC**: Using custom motor drivers with Arduino-SimpleFOC library
5. **Integration of Control Loops**: All control loops working in their layer of abstraction together
6. **Kinematic Integration**: Control of integrated mechanical system with kinematic integration
7.  **Incremental Testing**: For each hardware design iteration, there should be some testing done on the software side on a small unit/moddule of the hardware before we build the entire thing

## Project Documentation

### Code Repositories
- **High-Level Control**: `/high-level/src/` - State management and coordination
- **Low-Level Control**: `/low-level/src/` - Force control and motor interfaces  
- **Palm Interface**: `/palm/src/` - Multi-encoder sensor reading
- **GUI Applications**: `/GUI/` - Python control interfaces
- **Communication Library**: [rds25-comms](https://github.com/NU-RDS/rds25-comms) - For CAN bus communication and command parsing

### Reference Documentation
- **[Mechanical Documentation](https://docs.google.com/document/d/17M-Oa2aqqSKGwONml3L1aWp2K_8TE4KUtSQWIA2iMCw/edit?tab=t.yf60ihlam0il#heading=h.n3ni8fpki7n9)**: Specifications, CAD files, assembly drawings, kinematics
- **[Electrical Schematics](https://github.com/NU-RDS/rds25-project/blob/d32c9dea3d71abec86d8118c2d73201e1b604199/Electrical_Documentation/README.md)**: PCB designs and wiring diagrams (found in Electrical Documentation folder in repo)
- **[Calibration Data](https://docs.google.com/spreadsheets/d/170EN8GTMZlCyYZjw70axoC4KJDlu1CnwCoCHKkFP4lc/edit?usp=sharing)**: SEA force curves and sensor offsets
- **[Demos](https://drive.google.com/drive/folders/15svgBUCpVMmK7HvqHArqw5j0bLyojhVw?usp=drive_link)**: all the photos and videos taken during testing
- **[rds25-comms Documentation](https://github.com/NU-RDS/rds25-comms)**: Documentation for the CAN bus communication library, found in the readme.

## Getting Started

### Prerequisites
- PlatformIO for microcontroller development
- Python 3.8+ with matplotlib, numpy, tkinter
- CAN bus hardware (ODrive controllers)

### Quick Start
1. **Hardware Setup**: Connect encoders, load cells, and CAN devices according to electrical architecture
2. **Firmware Upload**: Flash appropriate code to each Teensy controller
3. **Calibration**: Run encoder offset and SEA force calibration procedures
4. **GUI Launch**: Execute `python GUI/PhaniGUI.py` for system control
5. **Testing**: Use preset configurations to verify system operation

### System Commands
```bash
# Compile and upload high-level controller
cd high-level && pio run -t upload

# Launch monitoring GUI  
python GUI/PhaniGUI.py

# Run force control validation
python GUI/SerialGUI.py
```

---

*For detailed technical documentation, calibration procedures, and troubleshooting guides, refer to the complete project repository and associated technical reports.*
