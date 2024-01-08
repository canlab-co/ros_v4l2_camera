# v4l2_camera

A ROS 2 camera driver using Video4Linux2 For Canlab (V4L2).

### System Requirements

Requirements:
  * CANLAB CLPE-G-01A
  * [ROS 2 Foxy](https://docs.ros.org/en/foxy/index.html)

### Download Pacakage
If you need to modify the code or ensure you have the latest update you will need to clone this repo then build the package.

    $ git clone --branch foxy https://github.com/canlab-co/ros_v4l2_camera.git
    $ colcon build

### Usage
Publish camera images, using the default parameters:

        ros2 launch v4l2_camera v4l2_camera_launch.py

Preview the image (open another terminal):

        ros2 run rqt_image_view rqt_image_view

## DDS Configuration
For better image transport performance over DDS, we recommend using [FastDDS](https://github.com/eProsima/Fast-DDS) with Shared Memory Transport enabled.
First copy the the `fastdds.xml` config file to a suitable directory, eg. `$HOME/fastdds.xml`
```bash
cd ~/ros_v4l2_camera/src
cp $HOME/fastdds.xml ~/
```

Next add these two lines to your `~/.bashrc`
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds.xml
```

Make sure to `source ~/.bashrc` first on all terminals before launching any ROS 2 nodes including the driver.

## Nodes

### v4l2_camera_node

The `v4l2_camera_node` interfaces with standard V4L2 devices and
publishes images as `sensor_msgs/Image` messages.

#### Published Topics

* `/image_raw` - `sensor_msgs/Image`

    The image.

#### Parameters

* `video_device` - `string`, default: `"/dev/video0"`

    The device the camera is on.

* `pixel_format` - `string`, default: `"UYVY"`

    The pixel format to request from the camera. Must be a valid four
    character '[FOURCC](http://fourcc.org/)' code [supported by
    V4L2](https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/videodev.html)
    and by your camera. The node outputs the available formats
    supported by your camera when started.  
    Currently supported: `"UYVY"`

* `output_encoding` - `string`, default: `"yuv422"`

    The encoding to use for the output image.  
    Currently supported: `"rgb8"`, `"yuv422"`.

* `image_size` - `integer_array`, default: `[1920, 1080]`

    Width and height of the image.

* Camera Control Parameters

    Not Support
