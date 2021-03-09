# Softserve simulation Common

## Node **move_base_route_manager**
This node was created to separate route manager logic and move_base navigation stack. It can be used as an example for the new adapter's creation.
All basic functionality is implemented on the base class RouteManager. 

### Configurable parameters  
There are several parameters to properly setup the route manager.
| Name          | Environment variable| Description   | Default  |
| ------------- | --- |-------------|--------:|
| goal_service | | service name to receive goals and send them to robot specific navigation stack | goal_service |
| goal_reached_topic_name | topic name, where result of goal execution is published | | goal_reached|

## Adapter requirements
1. Implements method **goal_execute(self, pose)**
2. Convert pose(PoseStamped) to navigation stack specific type
3. Method returns **True**, if the goal is reachable.
4. Method returns **False**, in all other situations.
5. Implement method what waits until goal is not reached. It have to return bool result

## New Adapter creation instruction
Please follow the next steps to create a new adapter:
1. Create a file for the adapter.
2. Create a class that is inherited from RouteManager
3. Initialize a base class during initialization
4. Implement the method **goal_execute(self, pose**
This method receives one argument:  
a) pose(type PoseStamped) - target pose  
5. For publishing current status of goal execution, call async function **self.goal_wait_async(arg)**. As an arg, it receives method(This method depends on navigation stack and should be implemented as well)) what checks if goal is reached by robot .
6. Implement all other functionality that is related to your navigation stack to fit requirements
