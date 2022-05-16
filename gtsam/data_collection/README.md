# Data collection
- The data collection in youbot is carried out using aruco markers placed on the floor of C025 lab.
- This data collection is done by rosbag command

```rosbag record -O subset /arm_cam3d/rgb/camera_info /arm_cam3d/rgb/image_raw /cmd_vel /map /odom /tf /tf_static```

The scp command used to copy bag file from youbot to the local system 

``` sudo scp robocup@youbot-brsu-4-pc2:/home/robocup/subset.bag /home ```

