🕷️ Hexapod-PicoW: Autonomous & Web-Controlled RobotA 12-DOF (Degree of Freedom) Hexapod robot controller powered by the Raspberry Pi Pico W. 

This project features a responsive Web Interface for control and telemetry, closed-loop stabilization using an IMU, and autonomous obstacle avoidance using Lidar.

✨ Key Features

🌐 Real-Time Web Telemetry: 

Hosts a responsive HTML5/JS dashboard serving orientation (Yaw/Pitch/Roll) and distance data via JSON.

🧠 Finite State Machine (FSM): 

Robust architecture handling states like WALKING, CALIBRATING, AVOIDING, and IMPACT_REACTION.

⚖️ Active Stabilization: 

Uses an MPU6050 to correct heading drift (PID-like proportional control) and detect physical impacts/crashes.

👀 Lidar Vision: Uses a VL53L0X ToF sensor for millimeter-precision obstacle detection and autonomous redirection.walk Engine: Oscillator-based inverse kinematics supporting multiple gaits:Tripod: Fast, stable 3-leg movement.Centipede: Wave-like motion for aesthetics.Sideways: Crab-walk capability.🛠️ Hardware ConfigurationThe code is designed for a 2-DOF per leg hexapod (12 Servos total).Component,Pin (GP),Function,Bus
PCA9685,GP8,SDA,Wire0
PCA9685,GP9,SCL,Wire0
MPU6050,GP8,SDA,Wire0
MPU6050,GP9,SCL,Wire0
VL53L0X,GP6,SDA,Wire1
VL53L0X,GP7,SCL,Wire1
Servo MappingThe robot assumes the following layout on the PCA9685 driver:Even Pins (0, 2, ... 10): Vertical Joints (Lift)Odd Pins (1, 3, ... 11): Horizontal Joints (Swing).

💻 Software Architecture

The system runs on a single-core loop structure optimized for the Arduino framework.1. 
The Class SystemHexapod: Handles low-level servo PWM and gait generation math (Triangle/Square waves).
IMU: Wrapper for the MPU6050. 
Handles gyro calibration, drift calculation, and impact detection algorithms.
LidarSensor: Manages the VL53L0X, filtering invalid readings and timeouts.
Timer: Non-blocking millis() wrapper for multitasking. 
State Machine DiagramThe robot logic flows through strict states to prevent conflicting commands:Boot -> STATE_MENU_GAIT (Wait for Web Command)Command 'walk' -> STATE_WALKINGIf Obstacle < 13cm: -> STATE_WAIT_TO_TURN -> STATE_AVOIDINGIf Crash Detected: -> STATE_IMPACT_REACTION (Recoil)Command 'stop' -> STATE_READY_TO_START🚀 
Installation & UsagePrerequisitesVS Code with PlatformIO (Recommended) OR Arduino IDE.
Board Support: Raspberry Pi Pico/RP2040 (Earle Philhower Core recommended).Library DependenciesAdd these to your platformio.ini or Arduino Library Manager:Adafruit PWMServoDriverPololu VL53L0XWireWiFiSetupConfigure Network:Edit the top of main.cpp:C++const char* ssid = "HEXAPOD_ROBOT";
const char* password = "YOUR_PASSWORD";
Upload: Connect the Pico W via USB and flash.Connect:Connect your phone/PC to the WiFi network HEXAPOD_ROBOT.
Open a browser to http://192.168.42.1.

🕹️ ControlsThe Web UI provides immediate feedback:Status 
Panel: Shows real-time Yaw/Pitch/Roll and current State.
Gaits: Switch between Tripod, Centipede, and Lateral on the fly.
Speed: Toggle between Slow, Medium, and Fast.
Calibration: Remotely trigger gyroscope zeroing if the robot drifts.
