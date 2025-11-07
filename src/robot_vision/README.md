# Robot Vision Package for ROS2

This package provides camera support for ROS2, converting the ROS1 roslaunch command to ROS2.

## Migration from ROS1 to ROS2

**ROS1 Command:**
```bash
roslaunch robot_vision robot_camera.launch device:=video0
```

**ROS2 Equivalent:**
```bash
ros2 launch robot_vision robot_camera.launch.py device:=video0
```

## Installation

### 1. Install Dependencies

First, install the `usb_cam` package for ROS2:

```bash
# For ROS2 Humble or later
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-usb-cam ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport

# Also install OpenCV for Python if needed
pip install opencv-python
```

### 2. Build the Package

```bash
cd ~/ros2workspace/ros2_ws
colcon build --packages-select robot_vision
source install/setup.bash
```

## Usage

### Basic Usage

Launch the camera with default settings (video0):
```bash
ros2 launch robot_vision robot_camera.launch.py
```

### Specify Device

Launch with a specific video device:
```bash
ros2 launch robot_vision robot_camera.launch.py device:=video0
```

Or with full path:
```bash
ros2 launch robot_vision robot_camera.launch.py video_device:=/dev/video0
```

### Advanced Usage

Customize camera parameters:
```bash
ros2 launch robot_vision robot_camera.launch.py \
    video_device:=/dev/video0 \
    image_width:=1280 \
    image_height:=720 \
    framerate:=30 \
    pixel_format:=mjpeg \
    camera_name:=front_camera
```

## Available Parameters

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `device` | `video0` | Short device name |
| `video_device` | `/dev/video0` | Full path to video device |
| `camera_name` | `robot_camera` | Camera name for topics |
| `frame_id` | `camera_link` | TF frame ID |
| `image_width` | `640` | Image width in pixels |
| `image_height` | `480` | Image height in pixels |
| `framerate` | `30` | Camera framerate (fps) |
| `pixel_format` | `yuyv` | Pixel format (yuyv, mjpeg) |

## Topics Published

The camera node publishes to the following topics:

- `/<camera_name>/image_raw` - Raw camera images
- `/<camera_name>/camera_info` - Camera calibration info

## Viewing Camera Output

To view the camera stream:

```bash
# Using rqt_image_view
ros2 run rqt_image_view rqt_image_view

# Or using RViz2
rviz2
```

Then select the appropriate image topic: `/robot_camera/image_raw`

## Troubleshooting

### Camera Not Found

If you get an error about the camera device not being found:

1. Check available video devices:
```bash
ls -l /dev/video*
```

2. Check camera permissions:
```bash
sudo usermod -a -G video $USER
# Log out and log back in for changes to take effect
```

3. Test camera with v4l2:
```bash
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

### Build Errors

If you encounter build errors, ensure all dependencies are installed:

```bash
cd ~/ros2workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robot_vision
```

## Key Differences from ROS1

1. **Launch Files**: Python-based instead of XML
2. **Command**: `ros2 launch` instead of `roslaunch`
3. **Package**: Uses `usb_cam` for ROS2 (different from ROS1 version)
4. **Parameters**: Passed with `:=` syntax, same as ROS1

## License

Apache-2.0
