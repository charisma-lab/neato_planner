# neato_planner
This package resides on each neato's raspberry pi. Using neato_localisation package, this package generates a path for the robot to follow.

### To test one point-to-point navigation using move_base:
roslaunch neato_planner waypoint_execution.launch

You can then take out the "test_waypoints" node from the launch file, and publish your own waypoint.
