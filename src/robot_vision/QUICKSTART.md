# Quick Start - Robot Vision Camera for ROS2

## 🚀 Quick Command Reference

### ROS1 to ROS2 Migration
```bash
# OLD (ROS1):
roslaunch robot_vision robot_camera.launch device:=video0

# NEW (ROS2):
ros2 launch robot_vision robot_camera.launch.py device:=video0
```

## 📋 Prerequisites

Install the USB camera package:
```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-usb-cam
```

## 🔧 Build & Source

```bash
cd ~/ros2workspace/ros2_ws
colcon build --packages-select robot_vision
source install/setup.bash
```

## 🎥 Launch Camera

**Basic (default /dev/video0):**
```bash
ros2 launch robot_vision robot_camera.launch.py
```

**Specify device:**
```bash
ros2 launch robot_vision robot_camera.launch.py video_device:=/dev/video0
```

**Custom resolution & framerate:**
```bash
ros2 launch robot_vision robot_camera.launch.py \
    video_device:=/dev/video0 \
    image_width:=1280 \
    image_height:=720 \
    framerate:=30
```

## 👁️ View Camera Output

**Method 1: rqt_image_view**
```bash
ros2 run rqt_image_view rqt_image_view
# Select topic: /robot_camera/image_raw
```

**Method 2: List & Echo topics**
```bash
ros2 topic list
ros2 topic echo /robot_camera/image_raw
ros2 topic hz /robot_camera/image_raw
```

## 🔍 Troubleshooting

**Check video devices:**
```bash
ls -l /dev/video*
```

**Fix permissions:**
```bash
sudo usermod -a -G video $USER
# Then logout and login again
```

**Test camera:**
```bash
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

## 📦 Package Contents

- `launch/robot_camera.launch.py` - Main launch file (ROS2 Python launch)
- `robot_vision/camera_node.py` - Custom camera node (optional)
- `package.xml` - Package dependencies
- `setup.py` - Python package setup

## 🎯 Key ROS2 Changes

| Aspect | ROS1 | ROS2 |
|--------|------|------|
| Launch command | `roslaunch` | `ros2 launch` |
| Launch file | XML `.launch` | Python `.launch.py` |
| Camera package | `usb_cam` (ROS1) | `usb_cam` (ROS2) |
| Discovery | Master-based | DDS-based |

## ✅ Quick Test

After launching, verify in a new terminal:
```bash
source ~/ros2workspace/ros2_ws/install/setup.bash
ros2 topic list | grep robot_camera
```

You should see topics like:
- `/robot_camera/image_raw`
- `/robot_camera/camera_info`
