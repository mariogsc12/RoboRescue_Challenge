# 🐢 RoboRescue UMA – MINI-PROJECT-1-2025

## ✨ Project Name: **ROBORESCUE Background Filler**

This repository contains our solution for the MINI-PROJECT-1-2025 challenge organized by RoboRescue UMA.

The goal of the project is to control the turtle in the turtlesim simulator using ROS 2 so that it draws the word **"ROBORESCUE"**. Our unique approach involves painting the **entire screen** and **leaving the letters unpainted**, making the name stand out against a filled background.

---

## 📁 Repository Contents

- `ros2_ws/`: ROS 2 package containing all source code.
- `utils/`: Bash script to open three terminals and launch each node for optimal visualization.
- `media/`: Video demonstrating the result.

---

## 🧠 Project Highlights

- ✅ Draws "ROBORESCUE" in the turtlesim window by excluding those regions from the background fill.
- ✅ Uses a modular architecture with clear separation between drawing logic, control loops, and utilities.
- ✅ Includes a `ros2 launch` and `.sh` file for ease of use.
- ✅ Use a YAML file to manage configuration parameters. 

---

## 🚀 How to Run the Project

### 1. Clone the repository and build the workspace:
```bash
git clone https://github.com/mariogsc12/RoboRescue_Challenge.git
cd RoboRescue_Challenge/ros2_ws/
colcon build
source install/setup.bash
```

### 2.1 Run the project using the provided bash script:
```bash
cd utils
bash launch.sh
```
### 2.2 Run the project using the launch script:
```bash
ros2 launch roborescue launch.launch.py
```

## 🛠️ Dependencies

  - ROS 2 Humble 

  - turtlesim package

  - GNOME Terminal for multi-terminal script
    ```bash
    sudo apt update
    sudo apt install gnome-terminal
    ```

## 🎥 Demo Video
...

## 🙌 Acknowledgments

Thanks to the RoboRescue UMA team for organizing this fun and educational challenge!
