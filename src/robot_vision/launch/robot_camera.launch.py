#!/usr/bin/env python3
"""
ROS2 Launch file for robot camera
Equivalent to: roslaunch robot_vision robot_camera.launch device:=video0

Usage:
  ros2 launch robot_vision robot_camera.launch.py device:=video0
  ros2 launch robot_vision robot_camera.launch.py video_device:=/dev/video0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='video0',
        description='Video device name (e.g., video0)'
    )
    
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Full path to video device'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='robot_camera',
        description='Camera name for topics'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='TF frame ID for the camera'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera framerate'
    )
    
    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='yuyv',
        description='Pixel format (yuyv, mjpeg, etc.)'
    )

    # Get launch configuration values
    video_device = LaunchConfiguration('video_device')
    camera_name = LaunchConfiguration('camera_name')
    frame_id = LaunchConfiguration('frame_id')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    pixel_format = LaunchConfiguration('pixel_format')

    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace=camera_name,
        parameters=[{
            'video_device': video_device,
            'image_width': image_width,
            'image_height': image_height,
            'framerate': framerate,
            'pixel_format': pixel_format,
            'frame_id': frame_id,
            'io_method': 'mmap',
        }],
        output='screen'
    )

    return LaunchDescription([
        device_arg,
        video_device_arg,
        camera_name_arg,
        frame_id_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        pixel_format_arg,
        usb_cam_node,
    ])
