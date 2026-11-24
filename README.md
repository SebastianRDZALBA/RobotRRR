# RobotRRR
3-DOF Robotic Arm (ESP32)

# 3-DOF Robotic Arm | ESP32 

A functional articulated robotic arm that integrates mechanical CAD design, electronics, embedded firmware, analytical robot kinematics, and a browser-based control interface.

The system uses an ESP32 to calculate joint and Cartesian positions, evaluate motion paths against mechanical constraints, and control the physical actuators through a local USB serial connection.

**Project type:** Mechatronics engineering academic prototype — Robot Architecture, 2026.

## Key features

- **Forward kinematics (FK):** calculates the Cartesian position (X, Y, Z) from the base, shoulder, and elbow angles.
- **Inverse kinematics (IK):** computes up to four analytical joint configurations for a requested Cartesian position and verifies them using forward kinematics.
- **Motion planning:** evaluates sequential and synchronized joint-space paths, including six possible joint update orders, before physical execution.
- **Mechanical constraints:** checks joint limits, sampled intermediate configurations, and implemented collision-related restrictions.
- **Homing and calibration:** establishes joint references using limit switches, supervised HOME routines, and servo calibration stored in ESP32 non-volatile storage (NVS).
- **Manual and automatic control:** supports individual joint movements, HOME and READY positions, and automatic gripper operation.
- **USB control interface:** provides live serial communication, estimated position telemetry, event logs, and operator confirmations.
- **Motion supervision:** includes software stop handling, reference invalidation after a stop, and a USB heartbeat watchdog.

## System architecture

The firmware is responsible for physical motion validation and execution. The web interface sends commands and displays the system state. Joint and Cartesian telemetry is estimated from commanded step counts and servo PWM rather than measured through continuous encoder feedback.

## Hardware and mechanical design

| Component | Function |
| --- | --- |
| ESP32 | Embedded controller, GPIO, PWM, UART and motion calculations |
| 2 × NEMA 17 stepper motors | Base and shoulder actuation |
| 2 × TMC2209 stepper drivers | STEP/DIR motor control |
| 2 × servomotors | Elbow and gripper actuation |
| Limit switches | Base and shoulder homing references |
| Buzzer | Audible notifications for HOME, READY and completed target movements |
| Custom mechanical assembly | Articulated structure, transmission, mounts and housing |

The three positioning degrees of freedom are provided by the base, shoulder and elbow. The gripper is a separate actuator and is not counted as a positioning degree of freedom.

### Mechanical model

| Parameter | Value |
| --- | ---: |
| Base height / L1 | 217 mm |
| Shoulder link / L2 | 105 mm |
| Elbow link / L3 | 122 mm |
| Lateral offset / Delta 2 | 33 mm |
| Lateral offset / Delta 3 | 23 mm |
| Combined signed lateral offset | -56 mm |
| Base and shoulder reduction | 3:1 |
| Stepper microstepping | 1/8 |

The kinematic model accounts for the lateral offsets of the actual mechanical assembly instead of assuming an ideal planar arm aligned with the base axis.

### CAD files

The mechanical design was developed in 3D and assembled as a physical prototype. STEP (`.step` / `.stp`) is the intended exchange format for sharing the CAD models.

## Firmware structure

The firmware is written in C using ESP-IDF, with responsibilities separated into reusable components.

```text
.
├── main/
│   └── main.c               # Initialization and command loop
├── components/
│   ├── base/                # Base movement and reference routines
│   ├── hombro/              # Shoulder movement and reference routines
│   ├── servos/              # Elbow and gripper PWM control
│   ├── driver_motores/      # Motor driver and step generation
│   ├── cinematica/          # Forward and inverse kinematics
│   ├── trayectorias/        # Path analysis and execution
│   ├── seguridad/          # Mechanical restrictions and stop handling
│   ├── home_ready/          # HOME and READY sequences
│   ├── comunicacion/        # Serial communication and UI commands
│   ├── estado_robot/        # Shared state and robot references
│   ├── buzzer/              # Audible notifications
│   └── config/              # GPIO, geometry, timing and limits
├── interfaz_usb/
│   ├── index.html           # Control dashboard
│   ├── styles.css           # Interface styling
│   ├── app.js               # Interface logic
│   ├── serial.js            # Web Serial communication
│   ├── iniciar_interfaz.py  # Local HTTP server
│   └── iniciar_interfaz.bat
├── CMakeLists.txt
└── sdkconfig
```

## Control interface

The dashboard is implemented with HTML, CSS and JavaScript. It runs locally without an internet connection and communicates with the ESP32 using the Web Serial API at 115200 baud.

The interface provides forward and inverse kinematics calculations, joint-space controls, gripper commands, HOME and READY operations, estimated telemetry, and a serial event console. It also has a simulated mode for exploring the interface without connecting the robot.

## Getting started

### Launch the web interface

On Windows, run `interfaz_usb/iniciar_interfaz.bat`, or use:

```bash
python interfaz_usb/iniciar_interfaz.py
```

Open `http://127.0.0.1:5517/`, select **Conectar USB**, and choose the ESP32 serial port. Complete the supervised HOME and READY procedures before commanding physical motion.

> **Hardware caution:** Verify wiring, supply ratings, joint clearances and calibration before operating the robot. The implemented software stop and motion restrictions are prototype safeguards, not a certified safety system. Removing actuator power or PWM may allow the arm to move under gravity.

## Engineering challenges and limitations

A major challenge was translating physical mechanical interference into constraints that could be checked by the firmware. The motion planner therefore evaluates intermediate joint configurations and rejects paths that violate the implemented rules.

The mechanical constraint model is not exhaustive: MOV2–MOV4 are retained in this firmware revision, while MOV1 is disabled. Complete restriction coverage across the workspace remains a future improvement. Position estimates also depend on homing, step counts and PWM calibration; they are not independent measurements of the physical end-effector position.

## Future improvements

- Complete and validate mechanical restriction coverage throughout the workspace, including the positive-X quadrants.
- Improve homing repeatability and position sensing for the stepper-driven joints.
- Improve cable routing and revise the housing to reduce mechanical interference.
- Add a protective grille for the cooling fan.
- Measure physical positioning accuracy and repeatability with external references.

## My contributions

- 3D mechanical design of the robotic arm.
- Physical assembly and mechanical integration.
- Electronic system design and actuator integration.

## Project status

Academic prototype. Further mechanical and control-system validation is required before any industrial use.
