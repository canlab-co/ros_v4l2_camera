# v4l2_camera

A ROS 2 camera driver using Video4Linux2 For Canlab (V4L2).

### System Requirements

Requirements:
  * CANLAB CLEB-G-Series [(GUIDE)](https://can-lab.atlassian.net/wiki/spaces/CANLABGUID/pages/485065636/CLEB-G-Series)
  * CANLAB CLV-G-Series [(GUIDE)](https://can-lab.atlassian.net/wiki/spaces/CANLABGUID/pages/453214214/CLV-G-Series)
  * CANLAB CLMU-G-Series [(GUIDE)](https://can-lab.atlassian.net/wiki/spaces/CANLABGUID/pages/484966555/CLMU-G-Series)
  * [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Download Pacakage
If you need to modify the code or ensure you have the latest update you will need to clone this repo then build the package.

    $ mkdir -p ~/ros_v4l2_camera/src
    $ cd ~/ros_v4l2_camera/src
    $ git clone --branch humble https://github.com/canlab-co/ros_v4l2_camera.git
    $ cd ~/ros_v4l2_camera
    $ colcon build
    $ source ~/ros_v4l2_camera/install/setup.bash

### Usage
Publish camera images, using the parameters:

        # launch the v4l2_camera executable
        CLV-G-Series : ros2 launch v4l2_camera v4l2_camera_clv_launch.py

        /* CLEB-G-01A */
        # CLCC-G-01X
        ros2 launch v4l2_camera v4l2_camera_cleb_launch.py image_size:="[1920, 1080]" cam:={n}
        # CLCC-G-02X
        ros2 launch v4l2_camera v4l2_camera_cleb_launch.py image_size:="[2048, 1280]" cam:={n}

        /* CLEB-G-02A */
        # CLSC-G-01X
        ros2 launch v4l2_camera v4l2_camera_cleb_launch.py image_size:="[1920, 1536]" cam:={n}

        /* CLMU-G-Series */
        # CLCC-G-01X
        ros2 launch v4l2_camera v4l2_camera_clmu_launch.py image_size:="[1920, 1080]" cam:={n}
        # CLCC-G-02X
        ros2 launch v4l2_camera v4l2_camera_clmu_launch.py image_size:="[2048, 1280]" cam:={n}

>Note: If the number of camera channels you want to use is 3, you can enter cam:=3.

        1CH camera (1 node)
        # run the executable with default settings:        
        ros2 run v4l2_camera v4l2_camera_node (default : /dev/video0, [1920, 1080])

        # run the executable with customized settings:
        ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video{x}" -p image_size:="[w, h]"

Preview the image (open another terminal):

        ros2 run rqt_image_view rqt_image_view

## DDS Configuration
For better image transport performance over DDS, we recommend using [FastDDS](https://github.com/eProsima/Fast-DDS) with Shared Memory Transport enabled.
First copy the the `fastdds.xml` config file to a suitable directory, eg. `$HOME/fastdds.xml`
```bash
cd ~/ros_v4l2_camera
cp fastdds.xml ~/
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
    Currently supported: `"rgb8", "rgba8", "bgr8", "bgra8", "mono16", "yuv422"`.  
    **Defaults to yuv422. Note that encodings other than yuv422 incurs conversion overhead.**
  
* `image_size` - `integer_array`, default: `[1920, 1080]`

    Width and height of the image.  
    Currently supported:  
    CLMU-G-Series - `[1920, 1080]` `[2048, 1280]`  
    CLEB-G-01A - `[1920, 1080]` `[2048, 1280]`  
    CLEB-G-02A - `[1920, 1536]`

* `cam` - `integer`

    The number of camera channels.  
    Currently supported:  
    CLMU-G-Series(default: `6`)  
    CLEB-G-Series(default: `6`)

* Camera Control Parameters

    Not Support
