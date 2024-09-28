# Object Follower

This project is a part of the larger Autonomous Vehicle initiative. BB-8, affectionately known as "Beebee-Ate," is a lovable and intelligent droid character from the __Star Wars__ franchise. Inspired by this iconic character, our robot BB-8 is designed to navigate and interact with its environment, showcasing advanced capabilities in object tracking and autonomous navigation.

## Overview

This ROS2 package enables a Turtlebot 3 to turn in place and follow an object. It consists of three main nodes that work together to process camera images, control the robot's movement, and provide debugging capabilities.

## Dependencies
- ROS2 Humble
- sensor_msgs
- std_msgs
- geometry_msgs

## Nodes
**1. find_object**

This node subscribes to the Raspberry Pi Camera topic /image_raw/compressed and processes the images to identify the location of an object. It then publishes the pixel coordinates of the object.
   
**2. rotate_robot**

This node subscribes to the object coordinate messages from the find_object node and publishes velocity commands to make the robot turn and follow the object.

**3. debug_node**

This node is designed to run on your personal computer and allows you to visualize the processed images from the Turtlebot.

## Usage
1. Ensure all dependencies are installed.
2. Clone this package into your ROS2 workspace.
3. Build the package using colcon build.
4. Source your workspace.
5. Run the nodes using:
```
ros2 run bb8_object_follower find_object
ros2 run bb8_object_follower rotate_robot
ros2 run bb8_object_follower debug_node
```

## Launch File

**Note:** The current launch file in this package may be incorrect. Please review and update it as necessary. Here's an example of what it might look like:
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bb8_object_follower',
            namespace='image_processor',
            executable='find_object',
            name='find_object_node'
        ),
        Node(
            package='bb8_object_follower',
            namespace='robot_control',
            executable='rotate_robot',
            name='rotate_robot_node'
        ),
        Node(
            package='bb8_object_follower',
            namespace='debug',
            executable='debug_node',
            name='debug_viewer_node'
        )
    ])
```

Ensure that the package name, executable names, and namespaces match your actual implementation.

## Camera Setup

To capture images from the Raspberry Pi camera, use the v4l2_camera_node:
```
ros2 run v4l2_camera v4l2_camera_node -ros-args -params-file ./v4l2_camera.yaml
```

## Tips
- Consider using launch files for easier startup of multiple nodes.
- Adjust QoS settings if experiencing network lag or dropped connections.
- For development and initial testing, you can use the Gazebo simulation environment.
