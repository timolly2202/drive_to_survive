# audibot_teleop_keys

`audibot_teleop_keys` is a lightweight ROS 2 package that enables manual keyboard control (teleoperation) of the **Audibot** robot via terminal input. It allows you to send throttle, steering, and braking commands over ROS topics using simple keystrokes.

---

## 🎮 What It Does

This node:
- Publishes velocity commands to the following topics:
  - `/orange/throttle_cmd` (`std_msgs/msg/Float64`)
  - `/orange/steering_cmd` (`std_msgs/msg/Float64`)
  - `/orange/brake_cmd` (`std_msgs/msg/Float64`)
- Accepts keyboard input in the terminal to control the robot:
  - `W/S`: Increase/decrease throttle
  - `A/D`: Steer left/right
  - `Spacebar`: Apply full brake
  - `R`: Reset all values to zero
  - `Q`: Quit

---

## 🧩 Dependencies

- ROS 2 (e.g., Humble)
- Python 3
- `rclpy`
- Terminal access (stdin must be available, not recommended for IDEs or terminals inside RViz)

---

## 🛠️ Build Instructions

Navigate to your ROS 2 workspace and build the package:

```bash
cd ~/drive_to_survive
colcon build --packages-select audibot_teleop_keys
source install/setup.bash
```

---

## 🚀 Running the Node

Make sure the Audibot simulator or system is running and ready to receive commands on the following topics:

- `/orange/throttle_cmd`
- `/orange/steering_cmd`
- `/orange/brake_cmd`

Then run:

```bash
ros2 run audibot_teleop_keys teleop_keys
```

You’ll see a printed menu in your terminal. Use the keyboard to control the robot.

---

## 🖥️ Notes

- You must run this from a real terminal. It won't work properly from within an IDE or graphical terminal emulator that doesn't pass raw key inputs.
- This package uses the Python `termios` module for low-level keyboard access.

---

## 👤 Author

**Jarred Deluca**  
📧 `jarred.g.deluca@student.uts.edu.au`

---

## 🛣️ Future Improvements

- Add configurable key bindings
- Integrate safety constraints (e.g., max speed/brake timeout)
- Support joystick or gamepad inputs