# Localization Tests

This package contains several tests for testing tools for a robot localization.

## Localization robot kidnapped test

The test verifies whether a localization tool localizes robot pose correctly when the estimated pose was reinitialized with wrong values. 
During the test, the robot navigates through the predefined or random points until the robot localised itself correctly or timeout occurred. 

### Prerequisites
For this test, the robot navigation stack should be run. Also, an adapter between the navigation stack and the test should be created and run. See an example of an adapter for move_base navigation in package [softserve_simulation_common](softserve_simulation_common/nodes/move_base_route_manager.py).

### How to run
```
roslaunch mp_localization_tests localization_kidnapped_test.launch
```
### Configurable parameters  
There are several parameters to properly setup the test.Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_LOCALIZATION_KIDNAPPED_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| start_time_seconds | ROBOT_LOCALIZATION_KIDNAPPED_TEST_START_TIME_SECONDS | time delay to start test in seconds | 10 |
| model_name | MODEL_NAME | robot model name in Gazebo | robot |
| environment     | ENVIRONMENT | environment, where test is run(AWS or LOCAL). | AWS  |
| localization_position_tolerance | ROBOT_LOCALIZATION_KIDNAPPED_TEST_POSITION_TOLERANCE | desired accuracy of robot estimated position in meters | 0.2 |
| localization_orientation_tolerance | ROBOT_LOCALIZATION_KIDNAPPED_TEST_ORIENTATION_TOLERANCE | desired accuracy of robot estimated yaw in radians | 0.2 |
| inflation_radius_coeff| | coefficient for robot inflation radius, can be needed for different robots | 1|
| occupancy_data_topic | | topic name with current map (msg type [OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) | map |
| map_metadata_topic | | topic with map metadata (msg type [MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)) | map_metadata |
| goal_service | | name of goal serviсe used in navigation stack adapter | goal_service |
| map_frame_id | | frame id what is used for pose publishing |map |
| mode | ROBOT_LOCALIZATION_KIDNAPPED_TEST_MODE | configure mode of route manager. **dynamic** - random goals on known map, **inorder** - goals from parameters used in order, **random** - goals from parameters, random order. | dynamic |
| initial_pose_topic_name | | topic where initial estimated pose is published (msg type [PoseWithCovarianceStamped](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) |/initialpose |
| localization_topic_name | | topic where estimated pose is published (msg type [PoseWithCovarianceStamped](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) |/amcl_pose |
| goal_reached_topic_name | | topic where status of navigation to goal is published (msg type [Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)) |goal_reached |
| navigation_success_count | ROBOT_LOCALIZATION_KIDNAPPED_TEST_NAVIGATION_SUCCESS_COUNT | number of goals |5|

### Test result
The test is marked as **Failed** if a timeout has occurred or predefined points were reached and robot did not localize itself with predefined accuracy.
The test is marked as **Passed** if a robot localizes itself with predefined accuracy within time, less than the timeout and all predefined points were not reached.  

### AWS tags  
During the test several tags could be set for a Robomaker Simulation Job.

|AWS tag| Description|
|---|---|
|Robot_Localization_Kidnapped_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Localization_Kidnapped_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Localization_Kidnapped_Test_<Timestamp>_Timed_Out| Is set when timeout occurs with timeout value |
|Robot_Localization_Kidnapped_Test_<Timestamp>_Failure_Reason| The reason why the test has been tagged as Failed |

## Localization test when the world's objects positions are changed

The test verifies whether a localization tool localizes the robot pose correctly when the predefined amount of world's objects are moved to positions 
that predefined in a yaml file for specific world. 
During the test, the robot navigates through the predefined or random points until the robot reached all points or timeout occurred. 

### Prerequisites
For this test, the robot navigation stack should be run. Also, an adapter between the navigation stack and the test should be created and run. See an example of an adapter for move_base navigation in package [softserve_simulation_common](softserve_simulation_common/nodes/move_base_route_manager.py).

### How to run
```
roslaunch mp_localization_tests localization_world_changes_test.launch
```
### Configurable parameters  
There are several parameters to properly setup the test.Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_LOCALIZATION_WORLD_CHANGES_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| start_time_seconds | ROBOT_LOCALIZATION_WORLD_CHANGES_TEST_START_TIME_SECONDS | time delay to start test in seconds | 10 |
| model_name | MODEL_NAME | robot model name in Gazebo | robot |
| environment     | ENVIRONMENT | environment, where test is run(AWS or LOCAL). | AWS  |
| localization_position_tolerance | ROBOT_LOCALIZATION_WORLD_CHANGES_TEST_POSITION_TOLERANCE | desired accuracy of robot estimated position in meters | 0.2 |
| localization_orientation_tolerance | ROBOT_LOCALIZATION_WORLD_CHANGES_TEST_ORIENTATION_TOLERANCE | desired accuracy of robot estimated yaw in radians | 0.2 |
| inflation_radius_coeff| | coefficient for robot inflation radius, can be needed for different robots | 1|
| occupancy_data_topic | | topic name with current map (msg type [OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) | map |
| map_metadata_topic | | topic with map metadata (msg type [MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)) | map_metadata |
| goal_service | | name of goal serviсe used in navigation stack adapter | goal_service |
| map_frame_id | | frame id what is used for pose publishing |map |
| mode | ROBOT_LOCALIZATION_WORLD_CHANGES_TEST_MODE | configure mode of route manager. **dynamic** - random goals on known map, **inorder** - goals from parameters used in order, **random** - goals from parameters, random order. | dynamic |
| localization_topic_name | | topic where estimated pose is published (msg type [PoseWithCovarianceStamped](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) |/amcl_pose |
| goal_reached_topic_name | | topic where status of navigation to goal is published (msg type [Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)) |goal_reached |
| navigation_success_count | ROBOT_LOCALIZATION_WORLD_CHANGES_TEST_NAVIGATION_SUCCESS_COUNT | number of goals |5|
| objects_count | ROBOT_LOCALIZATION_WORLD_CHANGES_TEST_OBJECTS_COUNT | number of objects that should be moved |5|

### Test result
The test is marked as **Failed** if a timeout has occurred or predefined points were reached and robot did not localize itself with predefined accuracy.
The test is marked as **Passed** if a robot localizes itself with predefined accuracy within time, less than the timeout or all points were reached and robot localizes itself with predefined accuracy.  

### AWS tags  
During the test several tags could be set for a Robomaker Simulation Job.

|AWS tag| Description|
|---|---|
|Robot_Localization_WorldChanges_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Localization_WorldChanges_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Localization_WorldChanges_Test_<Timestamp>_Timed_Out| Is set when timeout occurs with timeout value |
|Robot_Localization_WorldChanges_Test_<Timestamp>_Failure_Reason| The reason why the test has been tagged as Failed |

