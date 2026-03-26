Smart Explorer: PID-Controlled Maze Solver

An autonomous obstacle-avoidance robot featuring IMU-stabilized navigation, sector-scanning logic, and abnormality (stuck) detection.
🚀 Project Overview

This project involves a custom-built autonomous vehicle designed to navigate complex environments and solve mazes. Unlike basic "stop-and-turn" robots, this system utilizes Proportional-Integral-Derivative (PID) control for straight-line stability and an LSM6DSOX IMU for precise heading management.
Key Engineering Features:

    PID Heading Control: Uses gyroscope data to maintain a straight course, compensating for motor RPM mismatches.

    Stuck Detection (Abnormality Logic): A custom safety layer that monitors ultrasonic distance changes over time. If the robot is "moving" but the distance isn't changing, it triggers a recovery routine.

    Dynamic Sector Scanning: When an obstacle is detected, the robot performs a 180-degree sweep with a servo-mounted ultrasonic sensor to find the path with the highest clearance.

    Finish Line Detection: Integrated IR sensing for automatic termination upon reaching a target zone.

🛠 Hardware Stack

    Controller: Arduino (Nano 33 BLE Sense / compatible)

    IMU: LSM6DSOX (6-axis Inertial Measurement Unit)

    Actuators: 2x DC Motors with L298N Driver + MG90S Servo

    Sensors: HC-SR04 Ultrasonic Sensor, IR Flame/Line Sensor

    Power: Li-ion Battery Management System

🧠 Theory of Operation
1. Signal Processing & PID

The robot calculates its yaw (heading) by integrating the Z-axis gyroscope data.
Yaw=∫(GyroZ−Bias)dt

The error between the targetHeading and currentYaw is fed into a Proportional controller to adjust PWM signals for the left and right motors dynamically.
2. Abnormality Detection Logic

To solve the common "stuck in a corner" problem, the firmware implements a temporal check:

    Condition: If isMoving == true AND Time > 1.5s.

    Check: Is ΔDistance<2cm?

    Action: If true, the robot assumes a physical stall, backs up, and re-scans the environment.

💻 Development Methodology

    AI-Assisted Scripting: Utilized AI tools to accelerate boilerplate code generation and initial sensor library integration.

    Manual Hardware Tuning: Focused engineering efforts on calibrating the GYRO_SCALE, tuning the Kp gain for PID, and developing the recovery logic for physical edge cases.

    Phased Testing: Developed in stages (Basic Movement -> Sensor Fusion -> PID Tuning -> Anomaly Recovery).

📂 Repository Structure

    Smart_Explorer.ino: Main firmware .

    README.md: Project documentation.
