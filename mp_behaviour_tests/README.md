# Behaviour Tests

This package contains several tests for testing robot behavior, like navigation in the map, following route, etc.

## Navigation test
The navigation test shows the ability of the robot to navigate on a known map. As the acceptance criteria, the planned path length is used. 

### Prerequisites
For this test, the robot navigation stack should be run. Also, an adapter between the navigation stack and the test should be created and run. See an example of an adapter for move_base navigation in package [softserve_simulation_common](softserve_simulation_common/nodes/move_base_route_manager.py).

### How to run
```
roslaunch mp_behaviour_tests navigation_test.launch
```  
### Configurable parameters  
There are several parameters to properly setup the test. Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|:--------:|
| sim_timeout_seconds | ROBOT_NAVIGATION_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| environment     | ENVIRONMENT | environment, where test is run(AWS or LOCAL).  | AWS  |
| navigation_success_count | ROBOT_NAVIGATION_TEST_NAVIGATION_SUCCESS_COUNT | number of goals | 1 |
| distance_topic | | topic name, where distance to the goal is published (msg type [Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html))  | /distance_to_goal |
| goal_tolerance | ROBOT_NAVIGATION_TEST_GOAL_TOLERANCE | with which tolerance robot should reach every goal(path length) | 0.2 |
| path_topic | | the topic name, where Path from planner is published (msg type [Path](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html))| /move_base/NavfnROS/plan |
| inflation_radius | | the robot inflation radius | 0.4 |
| inflation_radius_coeff| | coefficient for robot inflation radius, can be needed for different robots | 1 |
| occupancy_data_topic | | topic name with current map (msg type [OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) | map |
| map_metadata_topic | | topic with map metadata (msg type [MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)) | map_metadata |
| map_frame_id | | frame id what is used for pose publishing |map |
| mode | ROBOT_NAVIGATION_TEST_MODE | configure mode of route manager. **dynamic** - random goals on known map, **inorder** - goals from parameters used in order, **random** - goals from parameters, random order. | dynamic |

### Test result
The test is marked as **Failed** if a timeout has occurred and the robot doesn't reach all navigation goals.
The test is marked as **Passed** if all navigation goal are reached within time, less than the timeout.  
  
### AWS tags  
During the test several tags could be set for AWS RoboMaker simulation job

|AWS tag| Description|
|---|---|
|Robot_Navigation_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Navigation_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Navigation_Test_<Timestamp>_Timed_Out| Is set when timeout occurs with timeout value |
|Robot_Navigation_Test_<Timestamp>_Successful_Nav_<SuccessGoalNumber>| Show, how many goals are reached within test |
|Robot_Navigation_Test_<Timestamp>_Failure_Reason| The reason why the test has been tagged as Failed |


## Coverage test

The coverage test shows the efficiency of the robot's ability to cover the floor of the environment. The test monitors robot movement and creates a coverage grid with places where the robot was already.
Based on this map and parameters, like tool_radius and coverage_area_offset, we are calculating coverage progress and comparing with it the coverage goal.

### Prerequisites
For this test, the robot should be switched to the mode corresponding to the full (or required part of ) map coverage, like in planner [FCPP](http://wiki.ros.org/full_coverage_path_planner#Overview)

### How to run
```
roslaunch mp_behaviour_tests coverage_test.launch
```  
### Configurable parameters  
There are several parameters to properly setup the test. Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_COVERAGE_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| environment     | ENVIRONMENT | environment, where test is run(AWS or not).  | AWS  |
| coverage_area_offset | | coverage area offset. Offset from map frame point 0 0 0 0 0 0 | -2.5 -2.5 0 0 0 0 |
| coverage_area_size_x | | coverage area height, X dimension of map  | 10 |
| coverage_area_size_y | | coverage area width, Y dimension of map | 10 |
| tool_radius | | robot tool radius  | 0.2 |
| coverage_goal | ROBOT_COVERAGE_TEST_COVERAGE_GOAL | coverage goal in percents | 80 |
  
### Test result
The test is marked as **Failed** if a timeout has occurred and the coverage progress is less than the coverage goal.
The test is marked as **Passed** if coverage goal is reached within time, less than the timeout.  
  
### AWS tags  
During the test several tags could be set for robomaker simulation job

|AWS tag| Description|
|---|---|
|Robot_Coverage_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Coverage_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Coverage_Test_<Timestamp>_Timed_Out| Is set when timeout occurs with timeout value |
|Robot_Coverage_Test_<Timestamp>_Current_Coverage| Coverage progress on time of timeout |


## Coverage effectiveness test

The coverage effectiveness test shows the efficiency of the robot's ability to cover the floor of the environment. The test monitors robot movement and creates a coverage grid with the resolution, which is equal to tool width what is equal to tool_radius * 2. The number of visits for each grid visits is taken into account

### Prerequisites
For this test, the robot should be switched to the mode corresponding to the full (or required part of ) map coverage, like in planner [FCPP](http://wiki.ros.org/full_coverage_path_planner#Overview)

### How to run
```
roslaunch mp_behaviour_tests coverage_effectiveness_test.launch
```  
### Configurable parameters  
There are several parameters to properly set up the test. Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_COVERAGE_EFFECTIVENESS_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| environment     | ENVIRONMENT | environment, where test is run(AWS or LOCAL).  | AWS  |
| coverage_area_offset | | coverage area offset. Offset from map frame point 0 0 0 0 0 0 | -2.5 -2.5 0 0 0 0 |
| coverage_area_size_x | | coverage area height, X dimension of map  | 10 |
| coverage_area_size_y | | coverage area width, Y dimension of map | 10 |
| tool_radius | | robot tool radius  | 0.2 |
| coverage_goal | ROBOT_COVERAGE_EFFECTIVENESS_TEST_COVERAGE_GOAL | coverage goal in percents | 80 |
| point_max_visits | ROBOT_COVERAGE_EFFECTIVENESS_TEST_POINT_MAX_VISITS | how many times, robot can visit cell in the grid   | 3 |
| max_deviation | ROBOT_COVERAGE_EFFECTIVENESS_TEST_MAX_DEVIATION | maximal deviation of the number of visits between cells in the grid | 1 |
| max_difference | ROBOT_COVERAGE_EFFECTIVENESS_TEST_POINT_MAX_DIFFERENCE | maximal difference of the number of visits between neighbor cells in the grid | 3 |
  
### Test result
The test is marked as **Failed** if a timeout has occurred and the coverage progress is less than the coverage goal or the robot has visited a single point more than the desired amount of times   
The test is marked as **Passed** if the coverage goal is reached within time, less than the timeout 
  
### AWS tags  
During the test, several tags could be set for AWS RoboMaker simulation job

|AWS tag| Description|
|---|---|
|Robot_Coverage_Effectiveness_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Coverage_Effectiveness_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Coverage_Effectiveness_Test_<Timestamp>_Timed_Out| Is set when a timeout occurs with timeout value |
|Robot_Coverage_Effectiveness_Test_<Timestamp>_Failure_Reason| The reason why the test has been tagged as Failed |


## Obstacle avoidance test

The obstacle avoidance test shows the planner's ability to replan trajectory to the goal if the obstacle is found on the original path. The test spawns obstacle on created path and checks if the robot reached the goal

### Prerequisites
For this test, the robot navigation stack and Gazebo simulator with a world should be run. Also, an adapter between the navigation stack and the test should be created and run. See an example of an adapter for move_base navigation in package [softserve_simulation_common](softserve_simulation_common/nodes/move_base_route_manager.py).
### How to run
```
roslaunch mp_behaviour_tests obstacle_avoidance_test.launch
```  
### Configurable parameters  
There are several parameters to properly set up the test. Some of them could be read from environment variables

| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| sim_timeout_seconds | ROBOT_OBSTACLE_AVOIDANCE_TEST_SIM_TIMEOUT_SECONDS | test timeout in seconds | 300 |
| environment     | ENVIRONMENT | environment, where the test is run(AWS or LOCAL).  | AWS  |
| path_length | | The minimal path length created by the planner to start a test | 10 |
| obstacle_sdf_file | | sdf file with obstacle model(all plugins should be deleted from model) | [model](../softserve_simulation_common/media/models/softserve_cube_50cm/model.sdf) |
| spawn_height | | the height, where obstacle will be spawned, in meters (depends on models) | 0.35 |
| path_topic | | the topic name, where Path from planner is published (msg type [Path](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html))| /move_base/NavfnROS/plan |
| inflation_radius | | the obstacle inflation radius | 0.4 |
| inflation_radius_coeff| | coefficient for obstacle inflation radius, can be needed for different obstacles | 1 |
| occupancy_data_topic | | topic name with current map (msg type [OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) | map |
| map_metadata_topic | | topic with map metadata (msg type [MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)) | map_metadata |
| map_frame_id | | frame id what is used for pose publishing | map |
| obstacle_count | OBSTACLE_COUNT | number of obstacles to spawn | 1 |
| obstacle_deviation | OBSTACLE_DEVIATION | obstacle position deviation (relatively to planner path, if value less then 0 - center of obstacle is located on the left side of path, value greater then 0 - on the right side) | 0.0 |

### Test result
The test is marked as **Failed** if a timeout has occurred and the robot didn't reach a goal
The test is marked as **Passed** if the robot reached a goal within time, less than the timeout 
  
### AWS tags  
During the test, several tags could be set for AWS RoboMaker simulation job

|AWS tag| Description|
|---|---|
|Robot_Obstacle_Avoidance_Test_<Timestamp>_Time_Elapsed_Start| Time when test started|
|Robot_Obstacle_Avoidance_Test_<Timestamp>_Status| Test result, Passed or Failed|
|Robot_Obstacle_Avoidance_Test_<Timestamp>_Timed_Out| Is set when a timeout occurs with timeout value |
|Robot_Obstacle_Avoidance_Test_<Timestamp>_Failure_Reason| The reason why the test has been tagged as Failed |
