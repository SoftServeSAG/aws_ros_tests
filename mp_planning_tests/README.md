# Planning Tests

This package contains several tests for testing robot navigation stack planning, like planning in the map, following plan, etc.

## Goal tolerance test
The goal tolerance test tests if the robot planner is configured properly. It measures the difference between the time when a goal is reached in the Gazebo world and the time when the robot planner reached a goal. 

### Prerequisites
For this test, the robot navigation stack should be run. Also, an adapter between the navigation stack and the test should be created and run. See an example of an adapter for move_base navigation in package [softserve_simulation_common](softserve_simulation_common/nodes/move_base_route_manager.py).

### How to run
```
roslaunch mp_planning_tests goal_tolerance.launch
```  

### Configurable parameters  
There are several parameters to properly setup the test. Some of them could be read from environment variables
| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|:--------:|
| sim_timeout_seconds | GOAL_TOLERANCE_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| model_name | MODEL_NAME | robot model name in Gazebo | robot |
| environment     | ENVIRONMENT | environment, where test is run(AWS or LOCAL).  | AWS  |
| goal_tolerance_time | GOAL_TOLERANCE_TEST_GOAL_TIME_TOLERANCE | max difference between the time when a goal is reached in the Gazebo world and the time when the robot planner reached a goal in seconds | 5 |
| goal_position_tolerance | GOAL_TOLERANCE_TEST_GOAL_POSITION_TOLERANCE | desired accuracy of robot position in meters | 0.2 |
| goal_orientation_tolerance | GOAL_TOLERANCE_TEST_GOAL_ORIENTATION_TOLERANCE | desired accuracy of robot yaw in radians | 0.2 |
| inflation_radius_coeff| | coefficient for robot inflation radius, can be needed for different robots | 1 |
| occupancy_data_topic | | topic name with current map (msg type [OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) | map |
| map_metadata_topic | | topic with map metadata (msg type [MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)) | map_metadata |
| goal_service | | name of goal serviсe used in navigation stack adapter | goal_service |
| map_frame_id | | frame id what is used for pose publishing | map |
| mode | GOAL_TOLERANCE_TEST_MODE | configure mode of route manager. **dynamic** - random goals on known map, **inorder** - goals from parameters used in order, **random** - goals from parameters, random order. | dynamic |

### Test result
The test is marked as **Failed** if a timeout has occurred and the robot doesn't reach a goal with the requested goal tolerance.
The test is marked as **Passed** if goal tolerance is reached within time, less than the timeout.  
  
### AWS tags  
During the test several tags could be set for AWS RoboMaker simulation job
|AWS tag| Description|
|---|---|
|Goal_Tolerance_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Goal_Tolerance_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Goal_Tolerance_Test_<Timestamp>_Timed_Out| Is set when timeout occurs with timeout value |
|Goal_Tolerance_Test_<Timestamp>_Failure_Reason| The reason why the test has been tagged as Failed |

## Navigation test (AB): robot navigates from point A to point B taking into account a goal yaw orientation

The test verify whether a robot navigates from point A to point B with predefined accuracy taking into account a goal yaw orientation.

### Prerequisites
For this test, the robot navigation stack should be run. Also, an adapter between the navigation stack and the test should be created and run. See an example of an adapter for move_base navigation in package [softserve_simulation_common](softserve_simulation_common/nodes/move_base_route_manager.py).

### How to run
```
roslaunch mp_planning_tests navigation_ab_test.launch
```
### Configurable parameters  
There are several parameters to properly setup the test.Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_NAVIGATION_AB_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| start_time_seconds | ROBOT_NAVIGATION_AB_TEST_START_TIME_SECONDS | time delay to start test in seconds | 10 |
| model_name | MODEL_NAME | robot model name in Gazebo | robot |
| environment     | ENVIRONMENT | environment, where test is run(AWS or LOCAL). | AWS  |
| goal_position_tolerance | ROBOT_NAVIGATION_AB_TEST_GOAL_POSITION_TOLERANCE | desired accuracy of robot position in meters | 0.2 |
| goal_orientation_tolerance | ROBOT_NAVIGATION_AB_TEST_GOAL_ORIENTATION_TOLERANCE | desired accuracy of robot yaw in radians | 0.2 |
| inflation_radius_coeff| | coefficient for robot inflation radius, can be needed for different robots | 1|
| occupancy_data_topic | | topic name with current map (msg type [OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) | map |
| map_metadata_topic | | topic with map metadata (msg type [MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)) | map_metadata |
| goal_service | | name of goal serviсe used in navigation stack adapter | goal_service |
| map_frame_id | | frame id what is used for pose publishing |map |
| mode | ROBOT_NAVIGATION_AB_TEST_MODE | configure mode of route manager. **dynamic** - random goals on known map, **inorder** - goals from parameters used in order, **random** - goals from parameters, random order. | dynamic |

### Test result
The test is marked as **Failed** if a timeout has occurred.
The test is marked as **Passed** if a robot navigated to point B with predefined accuracy within time, less than the timeout.  

### AWS tags  
During the test several tags could be set for a Robomaker Simulation Job.

|AWS tag| Description|
|---|---|
|Robot_Navigation_AB_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Navigation_AB_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Navigation_AB_Test_<Timestamp>_Timed_Out| Is set when timeout occurs with timeout value |


## Standstill test while driving from point A to point A 

The test checks if the robot planner is configured properly. It sends the current robot position as goal to planner and check whether robot is standstill with predefined accuracy. 

### Prerequisites
For this test, the robot navigation stack should be run. Also, an adapter between the navigation stack and the test should be created and run. See an example of an adapter for move_base navigation in package [softserve_simulation_common](softserve_simulation_common/nodes/move_base_route_manager.py).
The test assumes that the localization algorithm publishes the estimated pose the same as current robot pose in Gazebo. 
### How to run
```
roslaunch mp_planning_tests goal_tolerance.launch
```  

### Configurable parameters  
There are several parameters to properly setup the test. Some of them could be read from environment variables
| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|:--------:|
| sim_time_end_seconds | ROBOT_STANDSTILL_DRIVE_AA_TEST_SIM_TIME_END_SECONDS | predefined period of time in which the robot should be unmoved| 300|
| start_time_seconds | ROBOT_STANDSTILL_DRIVE_AA_TEST_START_TIME_SECONDS | time delay to start test in seconds | 10 |
| model_name | MODEL_NAME | robot model name in Gazebo | robot |
| environment     | ENVIRONMENT | environment, where test is run(AWS or LOCAL).  | AWS  |
| goal_position_tolerance | ROBOT_STANDSTILL_DRIVE_AA_TEST_GOAL_POSITION_TOLERANCE | desired accuracy of robot position in meters | 0.2 |
| goal_orientation_tolerance | ROBOT_STANDSTILL_DRIVE_AA_TEST_GOAL_ORIENTATION_TOLERANCE | desired accuracy of robot yaw in radians | 0.2 |
| inflation_radius_coeff| | coefficient for robot inflation radius, can be needed for different robots | 1 |
| occupancy_data_topic | | topic name with current map (msg type [OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) | map |
| map_metadata_topic | | topic with map metadata (msg type [MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)) | map_metadata |
| goal_service | | name of goal serviсe used in navigation stack adapter | goal_service |
| map_frame_id | | frame id what is used for pose publishing | map |

### Test result
The test is marked as **Failed** if the robot positions have been changed more than the specified accuracy and for less than the specified time for the test.
The test is marked as **Passed** if the robot was standstill during the predefined time for test.  
  
### AWS tags  
During the test several tags could be set for AWS RoboMaker simulation job
|AWS tag| Description|
|---|---|
|Robot_Standstill_Drive_From_A_To_A_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Standstill_Drive_From_A_To_A_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Standstill_Drive_From_A_To_A_Test_<Timestamp>_Timed_End| Is set when desired test duration occurs with time value |
