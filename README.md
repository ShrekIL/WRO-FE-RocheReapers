# RocheReapers Future Engineers Team

<p align="center">
  <img src="https://wro.innofabrik.de/wp-content/uploads/2021/08/logo-with-wordmark.png" alt="banner" width="1500">
</p>

This repository documents the development of our self-driving robot for the **Future Engineers 2025** division of the **World Robot Olympiad (WRO)**, created and coded by:

* [Guilherme Vercillo Fortes](https://github.com/vercillg)
* [Mischa Fässler](https://github.com/ShrekIL)
* [Robin Küttel](https://github.com/Robinkuet1)

<p align="center">
  <img src="https://wro.swiss/wp-content/uploads/2024/06/WRO_Icons_4-1200x796.png" alt="banner" width="1500">
</p>

The World Robot Olympiad (WRO) is a global robotics competition that inspires students around the world. Participants are challenged to demonstrate their creativity, problem-solving skills, and technical abilities by designing and programming robots for various tasks.

One exciting part of the WRO is the Future Engineers category. Here, students must develop innovative robotic and automated solutions to real-world problems. This category helps to cultivate future innovators by encouraging critical and creative thinking, which is essential for the next generation of engineers and technologists.

This year, the Future Engineers category includes an interesting challenge: building a self-driving car. This task requires participants to explore the latest advancements in robotics, adding complexity and innovation to the competition.

## Materials

| | Name                             | Image                                                                                                                                             |
| ---------| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------|
|| Raspberry Pi 5 8GB           | ![Raspberry-Pi5](https://ch.farnell.com/productimages/large/en_GB/4256000-500.jpg)                         |
|| Intel RealSense D435i           | ![Intel RealSense D435i](https://res.cloudinary.com/rsc/image/upload/b_rgb:FFFFFF,c_pad,dpr_2.625,f_auto,h_214,q_auto,w_380/c_pad,h_214,w_380/F1720981-03?pgw=1)              |
|| STL-19P TOF Lidar                      | ![STL-19P TOF Lidar](https://www.hiwonder.com/cdn/shop/files/2_1a530094-ef45-4053-89dc-dad4924d2b6e.jpg?v=1705919949%201200w)                         |
|| RCC lite Controller           | ![RCC lite Controlle](https://www.hiwonder.com/cdn/shop/files/1_39aaca4a-992b-4976-b21c-6a53e3ac99a9.jpg?v=1726816513)              |
|| Encoder Geared Motor| ![Encoder Geared Motor](https://www.hiwonder.com/cdn/shop/files/310_17.jpg?v=1717051806)
|| Chassis              | ![Chassis](https://www.hiwonder.com/cdn/shop/files/1.1_be4594b4-fa95-4fb4-bea3-f6d23ac7d03b.jpg?v=1705547494) |
|| Push Button              | ![Push Button](https://m.media-amazon.com/images/I/41vXmGH61EL._AC_.jpg) |
|| Spare pieces        | ![Spare pieces](https://cdn.shopify.com/s/files/1/0084/2799/5187/files/A1_3a64ef1b-7ae4-43e4-9bd1-38719fb0a00c.jpg?v=1730361141) |

## Brief description of the developed solution

#### 1. Problem Statement and Goal
The goal of the competition is to develop an autonomous vehicle that can complete two challenges:

- Open Challenge: Three laps on a track with a random layout (no obstacles).

- Obstacle Challenge: Three laps with colored obstacles (green = avoid left, red = avoid right) and a final parallel parking maneuver in a magenta-colored defined parking zone.

---

#### 2. System Overview
The vehicle is based on the Hiwonder MentorPi A1. To comply with WRO rules, one motor was removed, so the drive is now powered by a single motor mechanically coupled to a LEGO differential gear.
The originally installed 3D camera was replaced with an Intel RealSense D435i to achieve more precise color detection and improved ROS2 compatibility.
A Raspberry Pi 5 with 8 GB RAM is used as the main computer, combined with an RCC Lite Controller for motor and servo control.
The LiDAR sensor was relocated to the front of the vehicle to ensure the maximum vehicle height of 30 cm is not exceeded.
The servo steering had to be replaced due to a defect.

---

#### 3. Sensor Fusion and Environment Analysis
The LiDAR provides precise depth data for locating obstacles in the environment's coordinate system. The RealSense D435i handles color detection. Data from both sensors is processed and combined via ROS2 nodes, enabling the system to decide whether to avoid an obstacle on the left or right.

---

#### 4. Driving Strategy and Planning


---

#### 5. Parallel Parking


---

#### 6. Technical Challenges & Solutions
- Rule-Compliant Drive: Modified the MentorPi A1 to use a single drive motor with a LEGO differential gear.

- Camera Compatibility: Replaced the original camera with an Intel RealSense D435i with stable ROS2 support.

- Height Limit Exceeded: Relocated the LiDAR to the front of the vehicle to comply with the maximum height of 300 mm.

- Defective Steering: Replaced the defective servo with a new one.

- ROS2 Integration: All components are modularly networked via ROS2 nodes. This allows independent testing and easy system expansion.

## 🧩 Overview of the Code Structure
Our software is modular and follows a functional separation into sensing, perception, planning, and control. The main package is located in the `wro/` directory and is structured as a ROS2 Python package.

### 📁 Module Overview
| Module / File               | Function                                                    | Linked Hardware |
|----------------------------|-------------------------------------------------------------|---------------------|
| `main.py`                  | Entry point, initializes all modules                        | —                   |
| `block_vision_camera.py`   | Access and processing of RealSense D435i data               | Intel RealSense D435i |
| `block_vision.py`          | Color classification of obstacles (red/green)               | RealSense + Camera |
| `lidar.py`                 | LiDAR processing and obstacle detection                     | LiDAR (front-mounted) |
| `block_obstacle.py`        | Combination of LiDAR and color data into an object model    | Camera + LiDAR     |
| `obstacle.py`, `wall_obstacle.py` | Manage environmental obstacles (including static walls) | —                   |
| `path_planing.py`          | Decision: avoid left/right & parking logic                  | —                   |
| `dwa.py`                   | Dynamic Window Approach for trajectory generation           | —                   |
| `trajectory.py`            | Calculation and selection of trajectories                   | —                   |
| `stop.py`                  | Handling stop states (e.g., after 3 laps)                   | —                   |
| `utils.py`, `config.py`    | Helper functions and configuration values                   | —                   |
| `vision.py`                | Image processing and segmentation                           | Camera              |
| `kreis.py`, `icp.py`       | Geometric analyses (e.g., parking area, mapping)            | —                   |
| `wro.py`                   | Central control logic and state machine                     | All Modules         |

### 🧠 Structural Principle
- Sensing: `block_vision_camera.py`, `lidar.py` → Environment data acquisition
- Perception: `block_vision.py`, `block_obstacle.py`, `obstacle.py` → Interpretation
- Planning: `path_planing.py`, `dwa.py`, `trajectory.py` → Trajectory and avoidance
- Control: `stop.py`, `kreis.py`, `wro.py` → Decision logic, state handling
- Utilities: `utils.py`, `config.py`, `vision.py`, `icp.py`

## 🛠️ Build and Deployment Process

### 📦 1. System Requirements
- Raspberry Pi 5 (8GB)
- Ubuntu 22.04 with ROS 2 Humble (recommended)
- Python ≥ 3.10
- Intel RealSense SDK (for D435i)
- LiDAR driver (depending on model)
- RCC Lite Motor Controller (UART or USB communication)

### 🧰 2. Install Dependencies
```bash
# ROS 2 & colcon
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip

# RealSense SDK
sudo apt install librealsense2-dev

# Additional Python packages
pip3 install numpy opencv-python transforms3d
```

### 🏗️ 3. Set Up Workspace and Compile Code
```bash
mkdir -p ~/wro_ws/src
cd ~/wro_ws/src

git clone https://github.com/ShrekIL/WRO-FE-RocheReapers.git .

cd ~/wro_ws
colcon build
source install/setup.bash
```

### 🚀 4. Run Code
```bash
ros2 run wro main.py
```

### 🔁 5. Autostart (Optional)
To automatically start the program after booting:

1. Use a `.desktop` file or a `systemd` service.
2. Ensure `source ~/wro_ws/install/setup.bash` is executed beforehand.

### 🔌 6. Connection to RCC Lite Controller
Communication with the motor controller is done serially (e.g., `/dev/ttyUSB0`), configured in `config.py`.

## 🎥 Demonstration Videos