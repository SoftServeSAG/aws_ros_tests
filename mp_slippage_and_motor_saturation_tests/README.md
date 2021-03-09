# Slippage and motor saturation tests

This package contains several tests for testing slippage and motor saturation of a robot.


## Standstill test

The test has to detect whether the slippage is set to a realistic value. When standing on the flat surface the robot's 6DOF position should not change as time passes by.

### Prerequisites
In order to launch the test, Gazebo simulator with a world and the robot should be launched. 

### How to run
```
roslaunch mp_slippage_and_motor_saturation_tests standstill_test.launch
```

### Configurable parameters  
There are several parameters to properly setup the test.Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_STANDSTILL_TEST_SIM_TIME_END_SECONDS | predefined period of time in which the robot should be unmoved| 300 |
| start_time_seconds | ROBOT_STANDSTILL_TEST_START_TIME_SECONDS | time delay to start test in seconds | 10 |
| model_name | MODEL_NAME | name of the robot model in Gazebo | / |
| environment     | ENVIRONMENT | environment, where test is run(AWS or not). | AWS  |
| tolerance_position | ROBOT_STANDSTILL_TEST_TOLERANCE_POSITION | desired accuracy of the robot's position in meters | 0.01 |
| tolerance_orientation | ROBOT_STANDSTILL_TEST_TOLERANCE_ORIENTATION | desired accuracy of the robot's orientation | 0.01 |

### Test result
The test is marked as **Failed** if a robot's 6DOF position is unchanged during a predefined period of time. 
The test is marked as **Passed** if a robot's 6DOF position is changed within time, less than the predefined period of time.

### AWS tags
During the test several tags could be set for Robomaker Simulation Job.

|AWS tag| Description|
|---|---|
|Robot_Standstill_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Standstill_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Standstill_Test_<Timestamp>_Timed_Elapsed_End| Is set when desired test duration occurs with time value |

## Rotation test 

The test has to detect whether the yaw controller rotates the robot at a predefined angle. Robot rotation performed with IMU sensor should correlate with robot_state_publisher data within predefined accuracy level. AMCL is configured and published positions.

### Prerequisites
In order to launch the test, Gazebo simulator with a world and the robot should be launched.

### How to run
```
roslaunch mp_slippage_and_motor_saturation_tests rotation_test.launch
```

### Configurable parameters  
There are several parameters to properly setup the test.Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_ROTATION_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| start_time_seconds | ROBOT_ROTATION_TEST_START_TIME_SECONDS | time delay to start test in seconds | 10 |
| model_name | MODEL_NAME | name of the robot model in Gazebo | / |
| environment     | ENVIRONMENT | environment, where test is run(AWS or not). | AWS  |
| goal_angle | ROBOT_ROTATION_TEST_GOAL_ANGLE | desired robot's yaw in radians | 5.0 |
| angular_speed_z | ROBOT_ROTATION_TEST_ANGULAR_SPEED | rotation speed in radians/sec | 0.2 |
| tolerance| ROBOT_ROTATION_TEST_TOLERANCE | desired accuracy of the robot's yaw in radians | 0.1 |
| rotated_angle_topic|  | name of the topic where the current rotated yaw angle is published (msg type [Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)) | /rotated_angle |
| goal_angle_topic|  | name of the topic where the desired robot's yaw angle is published (msg type [Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)) | /goal_angle |
| amcl_pose_topic | | name of topic where AMCL position is published (msg type [PoseWithCovarianceStamped](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) | /amcl_pose |  

### Test result
The test is marked as **Failed** if a timeout has occurred and/or the rotated angle performed with IMU sensor does not correlate with robot_state_publisher data within predefined accuracy level.
The test is marked as **Passed** if the rotated angle performed with IMU sensor correlates with robot_state_publisher data within predefined accuracy level and  within time, less than the timeout.  

### AWS tags  
During the test several tags could be set for a Robomaker Simulation Job.

|AWS tag| Description|
|---|---|
|Robot_Rotation_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Rotation_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Rotation_Test_<Timestamp>_Timed_Out| Is set when timeout occurs with timeout value |


## Straight line test 

The test estimates how much does the robot's real trajectory deviates from the straight line. 

### Prerequisites
In order to launch the test, Gazebo simulator with a world and the robot should be launched.AMCL is configured and published positions.

### How to run
```
roslaunch mp_slippage_and_motor_saturation_tests straight_line_test.launch
```

### Configurable parameters  
There are several parameters to properly setup the test. Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_STRAIGHT_LINE_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| start_time_seconds | ROBOT_STRAIGHT_LINE_TEST_START_TIME_SECONDS | time delay to start test in seconds | 10 |
| model_name | MODEL_NAME | name of the robot model in Gazebo | / |
| environment     | ENVIRONMENT | environment, where test is run(AWS or not). | AWS  |
| distance| ROBOT_STRAIGHT_LINE_TEST_DISTANCE | desired moved distance in meters | 5.0 |
| linear_speed_x| ROBOT_STRAIGHT_LINE_TEST_LINEAR_SPEED_X | desired linear speed in meters/sec | 0.2 |
| tolerance| ROBOT_STRAIGHT_LINE_TEST_TOLERANCE | desired accuracy of the robot's trajectory in meters | 0.01 |
| moved_distance_topic|  | name of the topic where the current moved distance is published (msg type [Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)) | /moved_distance |
| goal_distance_topic|  | name of the topic where the desired moved distance is published (msg type [Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html)) | /goal_distance |
| amcl_pose_topic | | name of topic where AMCL position is published (msg type [PoseWithCovarianceStamped](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))| /amcl_pose |  

### Test result
The test is marked as **Failed** if a timeout has occurred and/or the trajectory of the robot movement does not correlate with straight line within the predefined accuracy level.
The test is marked as **Passed** if the robot traveled predefined distance with the trajectory which correlates with straight line within the predefined accuracy level and within time, less than the timeout.  


### AWS tags  
During the test several tags could be set for a Robomaker Simulation Job.

|AWS tag| Description|
|---|---|
|Robot_Straight_Line_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Straight_Line_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Straight_Line_Test_<Timestamp>_Timed_Out| Is set when timeout occurs with timeout value |
