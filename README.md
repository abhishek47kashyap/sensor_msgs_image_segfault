# Description
This repository is a minimum reproducible example of a segfault from accessing the `frame_id` of `sensor_msgs/Image`'s `header` field as received from a service server. This example was created and tested on ROS2 Humble.

This minimum reproducible example complements the question asked here: [Service response sensor_msgs/Image cannot be retrieved](https://robotics.stackexchange.com/questions/115197/service-response-sensor-msgs-image-cannot-be-retrieved)

The [client](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/main/src/client.cpp) makes a service request to obtain the newest RGBD that the [server](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/main/src/server.cpp) has on record. The service definition [`GetRGBD.srv`](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/main/srv/GetRGBD.srv) has `sensor_msgs/Image` for RGB and depth, and both of them have their `header.frame_id` corrupted(?) on the client side but not on the server side.

# Reproducing the segfault
## Prerequisites
- It's assumed ROS2 Humble is available and that `source /opt/ros/humble/setup.bash` is in `~/.bashrc`.
- It's assumed you're familiar with sourcing a ROS workspace and that every terminal we run commands in has to have the workspace (which we build in the next section) sourced.
- We will grab a RealSense D435i bag file ([sample recordings](https://github.com/IntelRealSense/librealsense/blob/master/doc/sample-data.md#files)) "_Outdoor scene with D435i pre-production sample (Depth from Stereo with IMU)_", it's the one at the very bottom of the table. Click on the image to its left or just click [here](https://librealsense.intel.com/rs-tests/TestData/d435i_sample_data.zip) to begin the download. It's a 451 MB zip file. Unzip it and there will be two bag files, we only need one so let's go with `d435i_walking.bag`.
## Commands to execute
### Cloning this repository and building the workspace
1. Create a workspace `mkdir -p segfault_ws/src && cd segfault_ws/src`
2. Clone this repository as `git clone https://github.com/abhishek47kashyap/sensor_msgs_image_segfault.git segfault_pkg`
3. Go back up one level `cd ..` and pull in dependencies with `rosdep update && rosdep install --from-paths src -y --ignore-src`. A standard ROS2 installation will already have most dependencies, the two you may not have could be [`realsense2_camera`](https://github.com/IntelRealSense/realsense-ros) for playing the bag file and [`backward_ros`](https://github.com/pal-robotics/backward_ros) which is great for pretty-printing the stack in the event of a segfault.
4. Build workspace with `colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

### Launching nodes and making the service request to observe segfault
We will use a total of three terminals, please source the workspace we just built in all of them.
1. In terminal 1, run `ros2 launch segfault_pkg server_client_launch.py`. Expect to see a log in green saying `Ready to accept trigger request!`.
2. Grab full filepath of the bagfile `d435i_walking.bag` and in terminal 2 run `ros2 launch realsense2_camera rs_launch.py rosbag_filename:='/full/path/to/d435i_walking.bag'`. In terminal 1 you should see these logs:
   ```
   First depth image rcvd!
   Acquired camera parameters!
   First color image rcvd!
   Acquired first RGBD frame
   First RGBD header frame_id: camera_color_optical_frame
   ```
   These logs confirm that [`server.cpp`](https://github.com/abhishek47kashyap/sensor_msgs_image_segfault/blob/main/src/server.cpp) had its subscription callbacks triggered by incoming messages from the bag file.
3. Finally, in terminal 3, ensure output of `ros2 service find std_srvs/srv/Trigger` produces `/example_segfault/trigger`. Then make the service request `ros2 service call /example_segfault/trigger std_srvs/srv/Trigger {}`. You will very likely observe a segfault in terminal 1 with the stacktrace printed out.

If however there is no segfault and you observe `CLIENT: SUCCESS` you're living a better life than me and I want to appreciate the time you took reading this far 🙂

## Comprehending the logs
In terminal 1, relevant logs generated on the server side have the prefix `SERVER` and those generated on the client side have `CLIENT`.

On the _server_ side, make note of:
```
Service get_rgbd was called!
SERVER: Frame ID: camera_color_optical_frame
SERVER: Data size: 921600
SERVER: Frame ID: camera_color_optical_frame
SERVER: size of color image height = 480, width = 640, encoding: rgb8, row length = 1920
SERVER: is_bigendian:
Completed get_rgbd service request, frame ID: camera_color_optical_frame
```

On the _client_ side, observe:
```
CLIENT: RGBD and camera pose acquisition status: true
CLIENT: Size of color image height = 480, width = 640, encoding: rgb8, row length = 1920
CLIENT: Data size: 921600
CLIENT: is_bigendian:
CLIENT: About to log header frame ID..
```

It's interesting that `frame_id` can be logged on the server side yet attempting to do so on client side causes a segfault. It's because of this segfault that `rgb` and `depth` cannot be copied over to another variable or forwarded on as part of another service request.
