# 🐢 RoboRescue UMA – MINI-PROJECT-1-2025

## ✨ Project Name: **ROBORESCUE Background Filler**

This repository contains our solution for the MINI-PROJECT-1-2025 challenge organized by RoboRescue UMA.

The goal of the project is to control the turtle in the turtlesim simulator using ROS 2 so that it draws the word **"ROBORESCUE"**. Our unique approach involves painting the **entire screen** and **leaving the letters unpainted**, making the name stand out against a filled background.

---

## 📁 Repository Contents

- `ros2_ws/`: ROS 2 package containing all source code.
- `media/`: Video demonstrating the result.

---

## 🧠 Project Highlights

- ✅ Draw "ROBORESCUE" in the turtlesim window by excluding those regions from the background fill.
- ✅ Use a modular architecture with clear separation between drawing logic, control loops, and utilities.
- ✅ Use turtlesim services to improve the turtle movement and word visualization
- ✅ Include a `ros2 launch` file for ease of use.
- ✅ Use a YAML file to manage configuration parameters. 

---

## 🚀 How to Run the Project

### 1. Clone the repository in the home and build the workspace:
```bash
cd
git clone https://github.com/mariogsc12/RoboRescue_Challenge.git
cd ~/RoboRescue_Challenge/ros2_ws/
colcon build
source install/setup.bash
```

### 2. Run the project using the launch script:
```bash
ros2 launch roborescue launch.launch.py
```

## 🛠️ Dependencies

  - ROS 2 Humble 

  - turtlesim package (http://wiki.ros.org/turtlesim)


## 🎥 Demo Video
...

## 🙌 Acknowledgments

Thanks to the RoboRescue UMA team for organizing this fun and educational challenge!
