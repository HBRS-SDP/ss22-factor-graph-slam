# Data collection using ArUco markers from YouBot
- The data collection in youbot is carried out using aruco markers placed on the floor of C025 lab.
- Set up the workspace in your local device by connecting to the YouBot through its SSH network
- Once connected, run the following commands to set up the workspace

    - ```tbringup``` in one terminal and
    - ```tplanning_bringup``` in another terminal
- Once both these terminals are executed , terminate both by `Ctrl+C` command. Then, run the following commands to launch the youbot in the workspace, 
  - ```bringup``` and ```planning_bringup``` in the respective terminated terminals
    - `bringup` is the alias command for launch file that executes all the necessary nodes for robot and arm movement
    - `planning_bringup` is the alias command for the launch file that executes all the necessary nodes for planning, and navigation (i.e SLAM) <br>


- By this time, the youbot should be launched and able to move with the joystick provided. Verify it by running the command `go candle` in the terminal (which moves the arm in the upright position)).

- Our purpose is to collect data from the youbot using the aruco markers placed on the floor. We need to localize the YouBot in the given map of the room. For this, we have an pre-built map of the room, available as a config file ([provided here](https://github.com/brsu-youbot/mas_industrial_robotics)) which helps the YouBot to localize itself.
 
 - Since, we are running the YouBot in our local, we need to export the ROS_MASTER_URI parameter by the following command
 ```ros 
 export ROS_MASTER_URI=http://<YouBot IP>:11311
 ```


 - Once the ROS_MASTER_URI is exported, launch `rviz` in the terminal. Before start recording the data, we need to set the map to the current map of the room by changing the config file ([provided here](https://github.com/brsu-youbot/mas_industrial_robotics)) in the `rviz` window.


 - Now, start recording the data by running the command and visualize the recording in the respective topics in the rviz window. We recorded the data by subscribing to multiple topics at the same time by using `-O`.

 ```ros
 rosbag record -O subset /arm_cam3d/rgb/camera_info /arm_cam3d/rgb/image_raw /cmd_vel /map /odom /tf /tf_static
 ```

 **NOTE: tf_static is the topic that contains the transform between the base_link and odom frames. This is required to get the transform between the base_link and the map frame.**

 - Export the ros bag file from youbot remote server to local device using the command,<br>
 ```ros 
 sudo scp robocup@youbot-brsu-4-pc2:/home/robocup/subset.bag /home
 ```
 - When the data is exported, run the command `go folded` in one of the YouBot terminals and then kill all the terminals.

# Aruco detection in the exported rosbag file

## Packages to be installed
- [aruco_detect](http://wiki.ros.org/aruco_detect)

## Do it in your local machine
- Execute `ros core` 


- **aruco_detect** package requires compressed image topics to be published by the camera. But, in the recorded rosbag file we have uncompressed image. So, we need to compress the rosbag file using the command, <br>
```ros 
<!-- rosrun image_transport republish compressed in:=/arm_cam3d/rgb/image_raw compressed out:=/device_0/sensor_1/Color_0/image -->
rosrun image_transport republish raw in:=/arm_cam3d/rgb/image_raw compressed out:=/device_0/sensor_1/Color_0/image
```

- Now, the compressed images are being published by ```/device_0/sensor_1/Color_0/image``` topic. Do the changes in aruco_detect launch file so that aruco_detect subscribes to this topic.
- Then, Launch aruco_detect using
```ros
roslaunch aruco_detect aruco_detect.launch
```
- Play the rosbag file in another terminal 
```ros 
rosbag play -l <compressed_out_filename>.bag
```
- Visualize the desired data in the terminal 



