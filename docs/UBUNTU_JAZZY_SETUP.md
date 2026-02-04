# Ubuntu 24.04 and ROS2 Jazzy Setup

This project is intended to run on **Ubuntu 24.04 (Noble)** with **ROS2 Jazzy Jalisco**.

## 1. Install ROS2 Jazzy on Ubuntu 24.04

Follow the official Debian package install:

- [ROS2 Jazzy - Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

Summary:

```bash
# Locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repo
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy desktop
sudo apt update
sudo apt install ros-jazzy-desktop

# Source in every shell where you use ROS2
source /opt/ros/jazzy/setup.bash
```

## 2. Optional: cv_bridge (for video_node image processing)

```bash
sudo apt install ros-jazzy-cv-bridge
```

## 3. Build this workspace

```bash
cd ~/ros   # or your workspace path
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Unitree G1 ROS2 stack (g1_platform, g1_interface)

If you run this app against a real G1, you need the Unitree G1 ROS2 packages (e.g. from Unitree’s documentation or their repository). Install and build those in a separate workspace or as dependencies, then ensure `g1_interface` and `g1_platform` are available and that the G1 driver is launched as described in the main [README](../README.md).

## 5. Persist environment (optional)

Add to `~/.bashrc` so every new terminal has ROS2 Jazzy and your workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros/install/setup.bash   # adjust path if needed
export ROS_DOMAIN_ID=10           # for G1
```

Then open a new terminal or run `source ~/.bashrc`.
