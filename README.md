# ROS2 Unitree Go1 Camera
This repository has code for interfacing with the Unitree Go1's onboard cameras with ROS2 Humble.

## Basic Usage
### Building the Package
The `unitree_camera` package relies on the `unitree_camera_launch_module` (in this repo, which defines custom Python launch substitutions), my fork of the [UnitreecameraSDK](https://github.com/ngmor/UnitreecameraSDK), and standard ROS2 Humble image processing packages (OpenCV 4, PCL 1.12, cv_bridge, etc).

To build:
```
mkdir -p ws/src
cd ws/src
git clone git@github.com:ngmor/unitree_camera.git
git clone git@github.com:ngmor/UnitreecameraSDK.git
cd ..
colcon build
```

### Running Nodes
To run all image publishers on the Go1's head Nano (192.168.123.13), run:

`ros2 launch unitree_camera head_publishers.launch.py`

To run all image publishers on the Go1's body Nano (192.168.123.14) run:

`ros2 launch unitree_camera body_publishers.launch.py`

To run all image subscribers, run:

`ros2 launch all_subscribers.launch.py`

Cameras and different image types can be enabled/disabled with input launch arguments to these launch files, listed [here](#launch-files).

## Nodes
### img_publisher
A generalized node which instantiates a `UnitreeCamera` object to read from the stereo cameras on the Go1. It can publish raw images, rectified images, depth images, and a point cloud. By default, only the rectified images are enabled.

#### Running
`ros2 run unitree_camera img_publisher`

#### Parameters
 - `fps` - int - The frame rate of the camera (fps).
 - `device_node` - int - Camera device node number. /dev/video0 is 0, /dev/video1 is 1.
 - `frame_width` - int - Camera frame width.
 - `frame_height` - int - Camera frame height.
 - `use_yaml` - bool - Configure camera with YAML file instead of parameters. MUST be true if using point cloud.
 - `yaml_path` - string - Path to yaml configuration file.
 - `enable_raw` - bool - Enable publishing of raw frames.
 - `enable_rect` - bool - Enable publishing of rectified frames.
 - `enable_depth` - bool - Enable publishing of depth frames.
 - `enable_point_cloud` - bool - Enable publishing point cloud data.
 - `point_cloud_frame` - string - Frame ID for point cloud messages.

### img_subscriber
A node used both for testing that images can be received and also as an example for how to subscribe to images.

#### Running
`ros2 run unitree_camera img_subscriber`

## Launch Files
Generally `ros2 launch unitree_camera ${launch_file} --show-args` can be used to list arguments for launch files, but arguments are also summarized here.


### General arguments
Several arguments are shared between multiple launch files with the same function.

`${cam}` in this documentation indicates which camera the argument affects. For example `${cam}.enable_cam` could be `head.front.enable_cam`, `bottom.left.enable_cam`, etc. Options for `${cam}` include:
- `head.front`
- `head.bottom`
- `body.left`
- `body.right`

Arguments:
- `${cam}.enable_cam` - Enable the `${cam}` camera
- `${cam}.enable_raw` - Enable publishing of raw frames from the `${cam}` camera.
- `${cam}.enable_rect` - Enable publishing of rectified frames from the `${cam}` camera.
- `${cam}.enable_depth` - Enable publishing of depth frames from the `${cam}` camera.
- `${cam}.enable_point_cloud` - Enable publishing point cloud data from the `${cam}` camera.
- `${cam}.point_cloud_frame` - Frame ID for point cloud messages from the `${cam}` camera. These default to the correct frames in the [Unitree Go1's URDF] (https://github.com/katie-hughes/unitree_ros2/tree/main/go1_description/xacro).

### head_publishers.launch.py
Launches image publishers for the Go1's head Nano (192.168.123.13).

`ros2 launch unitree_camera head_publishers.launch.py`

### body_publishers.launch.py
Launches image publishers for the Go1's body Nano (192.168.123.14).

`ros2 launch unitree_camera body_publishers.launch.py`

### head_subscribers.launch.py
Launches image subscribers for topics from the Go1's head Nano publishers.

`ros2 launch unitree_camera head_subscribers.launch.py`

### body_subscribers.launch.py
Launches image subscribers for topics from the Go1's body Nano publishers.

`ros2 launch unitree_camera body_subscribers.launch.py`

### all_subscribers.launch.py
Launches image subscribers for topics from all publishers.

`ros2 launch unitree_camera all_subscribers.launch.py`

### img_publisher.launch.py
General launch file for starting an `img_publisher` node specifically for one camera on the Go1.

`ros2 launch unitree_camera img_publisher.launch.py`

#### Arguments
- `camera` - Which camera to publish images from. Options are ['head_front', 'head_bottom', 'body_left', 'body_right']
- `enable_raw` - Enable publishing of raw frames.
- `enable_rect` - Enable publishing of rectified frames.
- `enable_depth` - Enable publishing of depth frames.
- `enable_point_cloud` - Enable publishing point cloud data.
- `point_cloud_frame`- Frame ID for point cloud messages.
