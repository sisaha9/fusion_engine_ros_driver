# Fusion Engine ROS 2 Driver

This is a C++ ROS 2 driver for [Point One Navigation](https://pointonenav.com) [Fusion Engine](https://pointonenav.com/fusionengine)

### Getting Started

##### Install Driver Dependencies

1. Install ROS 2: https://docs.ros.org/en/humble/index.html
2. 
```bash
git clone https://github.com/sisaha9/fusion_engine_client_ros2_driver.git
sudo apt update
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
rosdep install -y --ignore-src --from-paths .
```