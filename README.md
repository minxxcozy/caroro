# 🚗 Caroro Project 
> Autonomous Driving Competition Preparation with Simulator


## 🚀 Overview
**Caroro** is an autonomous driving project built on ROS (Robot Operating System).  
The system integrates multiple sensors and algorithms to handle real-world driving missions such as stop lines, obstacles, rotary intersections, and path planning.  


## 📦 Key Features
- **ROS-based Architecture**  
  Built on ROS for modular design and real-time communication between nodes.

- **Sensor Integration**  
  Utilizes LiDAR, camera, and odometry data for environment perception.

- **Perception Modules**  
  - Stop line detection  
  - Obstacle recognition and avoidance  
  - Rotary intersection handling  

- **Mission Handling**  
  Implements multi-phase driving logic such as stopping, lane following, turning, and merging.

- **Simulation & Real-world Deployment**  
  Supports both simulation environments and real-world vehicle platforms. 


## 🛠️ Tech Stack
- **Programming Language** : Python 3  
- **Framework** : ROS (Robot Operating System)  
- **Middleware** : ROS topics and messages for inter-node communication  
- **Computer Vision** : OpenCV for image processing (stop line detection, etc.)  
- **Path Planning** : Costmap and navigation stack (`wego_2d_nav`)  
- **Version Control** : Git & GitHub for collaborative development  


## ⚙️ Installation & Build
### 1️⃣ Clone repository :
```bash
git clone --recursive https://github.com/minxxcozy/caroro.git
cd caroro
```
### 2️⃣ Install dependencies :
```bash
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
pip install -r requirements.txt
```
### 3️⃣ Build workspace :
```bash
catkin_make
source devel/setup.bash
```


## 🎯 Run
### 1️⃣ Launch the MORAI Simulator
### 2️⃣ Run the launch file :
```bash
roslaunch
```

## 👥 Team Members
This project is developed by students from **Kookmin University, College of Automotive Engineering** :
- Yeonsil Kang  
- Minchae Kim  
- Woo In Jang  
- Hyeonjeong Jang  
- Heewon Choi  
