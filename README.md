# SSL_SLAM

3D SLAM using Solid-State LiDAR using ROS 2. This code is an updated implementation of paper "Lightweight 3-D Localization and Mapping for Solid-State LiDAR", accepted in IEEE Robotics and Automation Letters, 2021.

---

## Installation

| ROS DISTRO | Status |
|------------|--------|
| FOXY | ![build]() |
| HUMBLE | ![build]() |
| IRON | ![build]()|

```bash
cd ~/ros2_ws/src
git clone https://github.com/REGATTE/SSL_SLAM.git

cd ..
colcon build 
```

## Sensor Setup

The project uses a Solid State LiDAR, and uses the default solid state lidar available on NVIDIA ISAAC SIM. **Velarray M1600** and **Realsense L515** would be implemented soon. 

## Authors

- [J Ashok Kumar](https://github.com/REGATTE)

## 5. Citation
If you use this work for your research, you may want to cite the paper below, your citation will be appreciated 
```
@article{wang2021lightweight,
  author={H. {Wang} and C. {Wang} and L. {Xie}},
  journal={IEEE Robotics and Automation Letters}, 
  title={Lightweight 3-D Localization and Mapping for Solid-State LiDAR}, 
  year={2021},
  volume={6},
  number={2},
  pages={1801-1807},
  doi={10.1109/LRA.2021.3060392}}
```