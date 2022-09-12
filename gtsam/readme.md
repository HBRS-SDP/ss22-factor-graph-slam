## PlannerSLAM_using_collected_data

#### How to run the file
1. Clone and build the GTSAM library
2. Change the file path for collected data and where final final results to be stored
3. Format of Input data is shown below

| s.no | Frame_id | child_frame_id | landmark_x | landmark_y | landmark_z | Odom_x | Odom_y | Odom_r |
|------|----------|----------------|------------|------------|------------|--------|--------|--------|

4. Frame_id is 'base_link' of the robot
5. child_frame_id contains marker id in the environment
6. (landmark_x, landmark_y, landmark_z) - Aruco marker position with respect to 'base_link' of the robot
7. ( Odom_x, Odom_y, Odom_r) - Odometry data collected from the robot
8. Run the File

#### Process Flow

![program_work_flow drawio](https://user-images.githubusercontent.com/91040217/189561751-023282ad-863a-496e-9067-88157e7d552f.svg)
