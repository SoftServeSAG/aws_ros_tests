# ROS1 tests for AWS RoboMaker
The goal of this project is to provide multiple tests for the robot in simulation. The tests should be valid for robots of all kinds moving in 2D

Available tests:  
-   [Slippage and motor saturation tests](mp_slippage_and_motor_saturation_tests)
    -   [Standstill test](mp_slippage_and_motor_saturation_tests#standstill-test)
    -   [Rotation test](mp_slippage_and_motor_saturation_tests#rotation-test)
    -   [Straight line test](mp_slippage_and_motor_saturation_tests#straight-line-test)
-   [Planning tests](mp_planning_tests)  
    -   [Goal tolerance test](mp_planning_tests#goal-tolerance-test)  
    -   [Navigation AB test](mp_planning_tests#navigation-test-ab-robot-navigates-from-point-a-to-point-b-taking-into-account-a-goal-yaw-orientation)  
    -   [Standstill test](mp_planning_tests#standstill-test-while-driving-from-point-a-to-point-a)  
-   [Behaviour tests](mp_behaviour_tests)  
    -   [Navigation test](mp_behaviour_tests#navigation-test)
    -   [Coverage test](mp_behaviour_tests#coverage-test)
    -   [Coverage effectiveness test](mp_behaviour_tests#coverage-effectiveness-test)
    -   [Obstacle avoidance test](mp_behaviour_tests#obstacle-avoidance-test)
-   [Localization tests](mp_localization_tests)
    -   [Localization robot kidnapped test](mp_localization_tests#localization-robot-kidnapped-test)
    -   [Localization world changes test](mp_localization_tests#Localization-test-when-the-world's-objects-positions-are-changed)  
    