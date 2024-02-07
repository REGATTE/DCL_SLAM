# SSL_SLAM

3D SLAM using Solid-State LiDAR using ROS 2. This code is an updated implementation of paper "Lightweight 3-D Localization and Mapping for Solid-State LiDAR", accepted in IEEE Robotics and Automation Letters, 2021.

---

## Installation

| ROS DISTRO | Status |
|------------|--------|
| FOXY | ![build](https://github.com/REGATTE/SSL_SLAM/blob/main/.github/workflows/ros2_foxy/badge.svg) |
| HUMBLE | ![build](https://github.com/REGATTE/SSL_SLAM/blob/main/.github/workflows/ros2_humble/badge.svg) |
| IRON | ![build](https://github.com/REGATTE/SSL_SLAM/blob/main/.github/workflows/ros2_iron/badge.svg)|

Install all the dependencies needed

```bash
sudo apt install $(cat requirements.txt)
```

Source ros2 and its workspace, to the `~/.bashrc` file so that you dont have to source it everytime.
```bash
cd
nano ~/.bashrc
source opt/ros/{{distro}}/setup.bash
source ~/ros2_ws/install/setup.bash
```

Rename {{distro}} by whichever distro you're using.

```bash
cd ~/ros2_ws/src
git clone https://github.com/REGATTE/SSL_SLAM.git

cd ..
colcon build --symlink-install
```

*Make sure your setup-tools version is set to 52.8.0. This version is best for building packages.*

## Model Setup

 - Launch ISAAC SIM, with `ROS2_Bridge` activated.
 - 

The project uses a Solid State LiDAR, and uses the default solid state lidar available on NVIDIA ISAAC SIM. **Velarray M1600** and **Realsense L515** would be implemented soon. 

## Authors

- [J Ashok Kumar](https://github.com/REGATTE)
