# IMU-Controlled Robotic Arm

Embedded robotic-arm control system developed for ECE 167 at the University of California, Santa Cruz.

The complete project uses a wearable glove with an inertial measurement unit and flex sensor to translate hand motion into robotic-arm commands. The files currently included in this repository implement the STM32 servo-control subsystem, including PWM generation, servo-angle limits, mirrored arm movement, and a serial interface for manual testing.

## Project Overview

The robotic arm was designed to respond to wearable sensor input from a glove. The glove uses:

- An IMU to measure hand orientation and motion
- A flex sensor to detect finger bending
- Embedded firmware to process sensor measurements
- Real-time commands to control the robotic arm and gripper

The files currently included in this repository focus on the actuator-control portion of the system. They provide direct control of the following mechanisms:

- Base rotation
- Paired arm servos
- Top joint
- Gripper

The paired arm mechanism uses two servos mounted in opposite orientations. The firmware automatically mirrors the second servo command so both servos move the same mechanical joint together.

## Implemented Features

- PWM control for five physical servos
- Four logical control functions for the arm, base, top joint, and gripper
- Automatic mirrored movement for the paired arm servos
- Degree-to-PWM conversion
- Independent servo-angle limits
- Gripper-specific mechanical limits
- Serial commands for absolute positioning
- Serial commands for incremental positioning
- Servo status reporting
- Reset command that returns all servos to 90 degrees
- Input validation for unsupported commands
- PWM initialization error handling

## Servo Configuration

| Mechanism | Number of Servos | Commanded Range | Behavior |
|---|---:|---:|---|
| Arm | 2 | 0° to 180° | Servos move in mirrored directions |
| Base | 1 | 0° to 180° | Direct angular control |
| Top | 1 | 0° to 180° | Direct angular control |
| Gripper | 1 | 35° to 205° | Commands outside the allowed range are rejected |

The servo-control module maps commanded angles to PWM pulse widths between 500 microseconds and 2500 microseconds.

## PWM Outputs

The firmware initializes five PWM outputs for the robotic-arm servos.

The current servo assignments include:

| Mechanism | PWM Output |
|---|---|
| Primary arm servo | PWM output 1 |
| Mirrored arm servo | PWM output 2 |
| Base servo | PWM output 3 |
| Top servo | PWM output 4 |
| Gripper servo | PWM output 5 |

## Manual Servo Control

The main application provides a serial terminal interface for testing the robotic arm without requiring the glove input.

### Available Commands

| Command | Function |
|---|---|
| `a 90` | Set the paired arm servos to 90° |
| `a +5` | Increase the arm angle by 5° |
| `a -5` | Decrease the arm angle by 5° |
| `b 90` | Set the base servo to 90° |
| `t 90` | Set the top servo to 90° |
| `g 90` | Set the gripper servo to 90° |
| `s` | Display the current servo angles |
| `r` | Reset all servos to 90° |
| `h` | Display the command list |

The full command names `status`, `reset`, and `help` are also accepted.

## Software Structure

### ForceFlexGripper.c

Contains the main application and serial command interface.

Responsibilities include:

- Initializing the board, timers, and servo module
- Reading commands from the serial terminal
- Converting command text to lowercase
- Parsing servo identifiers and angle values
- Supporting absolute and relative angle commands
- Printing servo status
- Resetting all servos
- Reporting missing values and unknown commands

### Servo.c

Contains the servo-control implementation.

Responsibilities include:

- Initializing the PWM hardware
- Configuring the five PWM outputs
- Setting timer periods
- Converting angles into PWM pulse widths
- Restricting arm, base, and top commands to valid ranges
- Rejecting unsafe gripper commands
- Generating mirrored commands for the paired arm servos
- Storing the current commanded angle for each mechanism

### Servo.h

Defines the public servo-control interface used by the rest of the project.

The interface includes functions for:

- Initializing the servo subsystem
- Setting the arm angle
- Setting the top-joint angle
- Setting the base angle
- Setting the gripper angle
- Reading the current commanded angle for each mechanism

## Initialization

The application initializes the board, timers, and servo subsystem before accepting control commands.

During startup, the arm, base, top joint, and gripper are each commanded to a neutral position of 90 degrees.

## Angle-to-PWM Conversion

The servo module converts requested joint angles into PWM pulse widths.

The conversion process:

1. Checks the requested angle against the servo’s valid range
2. Converts the angle into a pulse width
3. Writes the pulse width to the corresponding timer output
4. Stores the latest commanded angle for status reporting

The configured pulse-width range is 500 to 2500 microseconds.

## Mirrored Arm Control

The arm mechanism uses two servos mounted in opposite orientations.

The primary servo receives the normal PWM command, while the second servo receives a mirrored command. This allows the two servos to move together in the same mechanical direction despite being installed in opposite orientations.

## Gripper Protection

The gripper has a restricted operating range of 35° to 205°.

Commands outside this range are rejected and reported through the serial terminal. This prevents the firmware from intentionally commanding the gripper beyond its defined mechanical limits.

## Dependencies

The current source files depend on board-support and course libraries for:

- Board initialization
- Timer initialization
- PWM configuration
- Serial communication

The project also uses standard C libraries for string handling, command parsing, and terminal output.

## Repository Files

- `ForceFlexGripper.c` — main application and serial command interface
- `Servo.c` — servo-control and PWM implementation
- `Servo.h` — servo-control interface
- `README.md` — project documentation

## My Contributions

I served as the Embedded Systems Lead for this project. My work included:

- Developing the servo-control firmware
- Configuring PWM outputs for the robotic arm
- Implementing angle-to-PWM conversion
- Coordinating the mirrored arm servos
- Defining servo operating limits
- Protecting the gripper from out-of-range commands
- Creating the serial testing interface
- Supporting absolute and incremental servo commands
- Debugging robotic-arm movement
- Integrating wearable sensor commands with robotic actuation
