# Lab 5 Fall 2025 - Neural Controller Configuration with Wheel Driving Mode

This directory contains configuration files for the Pupper v3 neural controller and a deployment script that pushes changes and rebuilds the ROS2 workspace. **NEW: Includes wheel driving mode with automatic leg transition!**

## Files

- **`config.yaml`** - Neural controller configuration file with settings for all controller modes (normal, three-legged, parkour, and wheel driving)
- **`launch.py`** - ROS2 launch file that starts the neural controller with all necessary nodes including wheel driving support
- **`estop_controller.cpp`** - Emergency stop controller C++ source code for switching between controllers
- **`parkour_policy.json`** - Neural network policy file for parkour mode (deployed to neural_controller/launch/)
- **`hub_motor_controller.py`** - NEW: ROS2 node for controlling hub motors via USB serial
- **`mode_transition_controller.py`** - NEW: ROS2 node for transitioning between walking and driving modes
- **`rebuild_neural_controller.py`** - Deployment and rebuild script (copies files + runs build.sh + sets up wandb)
- **`download_latest_policy.py`** - Downloads a specific policy from Weights & Biases (auto-detects user entity)
- **`deploy.py`** - Interactive script to download a policy (optional) and launch the robot

## NEW FEATURE: Wheel Driving Mode 🚗

Your Pupper can now switch between **walking mode** (using end effectors) and **driving mode** (using hub motors/wheels)!

### How It Works

1. **Walking Mode**: Default mode where Pupper walks using its neural controller with end effectors touching the ground
2. **Driving Mode**: All four legs rotate 90° to position wheels on the ground:
   - **Front wheels**: Powered hub motors drive the robot forward/backward and turn
   - **Back wheels**: Omni wheels provide passive support and allow smooth turning
3. **Seamless Transition**: Press a button on your remote controller to switch between modes

### Hardware Setup

1. Connect the hub motor driver board to Raspberry Pi via USB (should appear as `/dev/ttyUSB0`)
2. Ensure **front 2 hub motors** (left and right) are properly connected to the driver board
3. Verify **back 2 omni wheels** are installed and can rotate freely (no motor connection needed)
4. Ensure motors are powered and functional

### Controller Button Mapping

- **Button 4** (default): Switch between walking and driving modes
- During transition: All four legs rotate to bring wheels into position (takes 3 seconds)
  - Front legs position **powered hub motors** on ground
  - Back legs position **omni wheels** on ground for stability
- In driving mode: 
  - Front wheels drive and steer the robot (differential drive)
  - Back omni wheels passively roll and allow turning
  - Use joystick for direction control (left stick for forward/backward, right stick for turning)

## Usage

### Quick Start: Launch the Robot with Wheel Support

To download a policy (optional) and launch the neural controller with wheel driving enabled:

```bash
python3 deploy.py
```

This interactive script will:
1. Prompt you for a run number to download a specific policy from wandb (or press Enter to skip)
2. Download the policy if a run number is provided (automatically uses your logged-in wandb entity)
3. Launch the neural controller with wheel driving mode enabled

### Manual Launch

```bash
# Launch with wheel driving enabled (default)
ros2 launch neural_controller launch.py enable_wheels:=True

# Launch without wheel driving (walking only)
ros2 launch neural_controller launch.py enable_wheels:=False

# Launch in simulator with wheels
ros2 launch neural_controller launch.py sim:=True enable_wheels:=True
```

### Switching Between Modes

**While running:**

1. Start in walking mode (neural controller active)
2. Press **Button 4** on your joystick to transition to driving mode
   - Robot will rotate all four legs (3 second transition)
   - Hub motors will activate automatically
   - Use joystick to drive with wheels
3. Press **Button 4** again to return to walking mode
   - Robot will rotate legs back to walking position
   - Neural controller reactivates

### Deploy and Rebuild

To deploy configuration files (including new wheel driving nodes) and rebuild:

```bash
# Preview what will be done (dry run - no files modified, no build)
python3 rebuild_neural_controller.py --dry-run

# Deploy files and rebuild the workspace
python3 rebuild_neural_controller.py

# Deploy only without rebuilding
python3 rebuild_neural_controller.py --no-build
```

The script will now also deploy:
- `hub_motor_controller.py` → `/home/pi/pupperv3-monorepo/ros2_ws/src/neural_controller/scripts/`
- `mode_transition_controller.py` → `/home/pi/pupperv3-monorepo/ros2_ws/src/neural_controller/scripts/`

## Configuration Overview

### Robot Modes

1. **WALKING** - Default neural controller mode (end effectors on ground)
2. **DRIVING** - Wheel mode (hub motors on ground)
3. **TRANSITIONING** - Intermediate state during mode change

### Controller Modes

The configuration supports four neural controller modes (all for walking):

1. **neural_controller** - Default mode using `policy_latest.json`
2. **neural_controller_three_legged** - Three-legged locomotion mode
3. **neural_controller_parkour** - Parkour mode using `parkour_policy.json`
4. **joint_trajectory_controller** - Used during mode transitions

All controllers start inactive and are activated automatically by the mode transition controller.

### Launch Arguments

- `sim:=True/False` - Run in Mujoco simulator (True) or on real robot (False, default)
- `teleop:=True/False` - Enable teleoperation (True, default)
- `enable_wheels:=True/False` - Enable wheel driving mode (True, default)

Example:
```bash
ros2 launch neural_controller launch.py sim:=False teleop:=True enable_wheels:=True
```

### Hub Motor Controller Parameters

Located in `config.yaml`:

```yaml
hub_motor_controller:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"  # USB serial port for motor driver
    baud_rate: 115200            # Serial communication speed
    wheel_base: 0.2              # Distance between front left and right wheels (meters)
    wheel_radius: 0.05           # Front powered wheel radius (meters)
    max_rpm: 100                 # Maximum motor speed
    # Note: Only front 2 wheels are powered
    # Back 2 wheels are omni wheels (passive)
```

### Mode Transition Parameters

```yaml
mode_transition_controller:
  ros__parameters:
    transition_duration: 3.0     # Time to rotate legs (seconds)
    transition_button: 4         # Joystick button for mode switch
```

## Typical Workflow

### First Time Setup with Wheels
1. Connect hub motor driver board via USB
2. Edit configuration files as needed
3. Deploy and rebuild: `python3 rebuild_neural_controller.py`
4. Authenticate with wandb when prompted
5. Launch the robot: `python3 deploy.py`
6. Test mode transition with Button 4

### Daily Usage with Wheels
1. Launch robot: `python3 deploy.py`
2. Start in walking mode (default)
3. Press Button 4 to switch to driving mode
4. Drive around with wheels
5. Press Button 4 to return to walking mode

### Testing Wheel Driving Only
```bash
# Launch with wheels enabled
ros2 launch neural_controller launch.py enable_wheels:=True

# In another terminal, manually trigger driving mode
ros2 topic pub /robot_mode std_msgs/String "data: 'DRIVING'" --once

# Control wheels directly
ros2 topic pub /cmd_vel_wheels geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## Troubleshooting

### Hub Motors Not Responding
1. Check USB connection: `ls /dev/ttyUSB*`
2. Verify serial port in config: `/dev/ttyUSB0`
3. Check motor driver board power
4. View logs: `ros2 node list` and check `hub_motor_controller`

### Transition Not Completing
1. Ensure robot has clearance for leg movement
2. Check transition duration (increase if needed)
3. Verify joint limits in URDF
4. Monitor joint states: `ros2 topic echo /joint_states`

### Mode Switch Button Not Working
1. Verify joystick connection: `ros2 topic echo /joy`
2. Check button index in config (default: button 4)
3. Ensure `joy_node` is running

## Directory Structure

```
/home/pi/lab_5_fall_2025/           # Source files (this directory)
├── config.yaml                      # Controller configuration
├── launch.py                        # Launch file with wheel support
├── estop_controller.cpp             # Emergency stop controller source
├── parkour_policy.json              # Parkour neural network policy
├── hub_motor_controller.py          # NEW: Hub motor control node
├── mode_transition_controller.py    # NEW: Mode transition node
├── rebuild_neural_controller.py     # Deployment and rebuild script
├── download_latest_policy.py        # Download policy from wandb
├── deploy.py                        # Launch robot with optional policy download
└── README.md                        # This file

/home/pi/pupperv3-monorepo/ros2_ws/src/neural_controller/  # Destination
├── launch/
│   ├── config.yaml                  # Deployed configuration
│   ├── launch.py                    # Deployed launch file
│   └── parkour_policy.json          # Deployed parkour policy
└── scripts/
    ├── hub_motor_controller.py      # Deployed hub motor controller
    └── mode_transition_controller.py # Deployed mode transition controller

/home/pi/pupperv3-monorepo/ros2_ws/src/joy_utils/src/  # Destination
└── estop_controller.cpp             # Deployed emergency stop controller

/home/pi/pupperv3-monorepo/ros2_ws/  # Build location
└── build.sh                         # Build script (automatically executed)
```

## ROS Topics Reference

### Published by Wheel System
- `/robot_mode` (std_msgs/String) - Current robot mode (WALKING/DRIVING/TRANSITIONING)
- `/hub_motors/enable` (std_msgs/Bool) - Hub motor enable state

### Subscribed by Wheel System
- `/joy` (sensor_msgs/Joy) - Joystick input for mode switching
- `/cmd_vel_wheels` (geometry_msgs/Twist) - Velocity commands for hub motors

### Internal Topics
- `/joint_trajectory_controller/joint_trajectory` - Joint commands during transition
- `/joint_states` - Current joint positions and velocities

## Notes

- Backup files are automatically created with `.backup` extension
- The rebuild script checks for file existence before copying
- The ROS2 workspace is automatically rebuilt after deployment
- Hub motor control is separate from neural controller - they don't run simultaneously
- During transition, robot is in position control mode (not neural network)
- Always ensure adequate clearance when switching modes
- The mode transition controller automatically handles controller activation/deactivation
- Serial communication protocol can be customized in `hub_motor_controller.py`
- Default transition takes 3 seconds - adjust `transition_duration` if needed
- Use `--no-build` flag if you only want to deploy without rebuilding

## Safety

- **Emergency Stop**: Use estop controller as usual (works in both modes)
- **Mode Transition**: Ensure robot has space to rotate legs
- **Wheel Speed**: Limited by `max_rpm` parameter (default: 100 RPM)
- **Watchdog Timer**: Hub motors auto-stop after 0.5s without commands

## Future Enhancements

- [ ] Add position feedback from hub motors
- [ ] Implement closed-loop speed control
- [ ] Add battery voltage monitoring
- [ ] Create hybrid mode (walking + driving simultaneously)
- [ ] Add autonomous mode switching based on terrain
- [ ] Integrate wheel odometry for localization