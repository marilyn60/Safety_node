## The Safety Node’s purpose

This is a ROS 2 node for an F1TENTH race car, following Lab 2 of RoboRacer autonomous vehicles. The purpose of this node is automatic emergency braking, when it gets close to an obstacle it stops at a safe distance.

##How it works:

The node listens to `/scan` (the LiDAR topic) and `/ego_racecar/odom` (the odometry topic). For each LiDAR beam, it calculates how many seconds it will take for a collision to occur if the car does not stop, using time of collision = distance / closing speed. It keeps the smallest time, and if it is under 0.5 seconds the node publishes `speed = 0.0` to `/drive` and the car stops. To avoid false braking, it skips beams the car is moving away from, since the distance is growing, and invalid readings, such as being out of sensor range.

##How it runs:

It runs on top of the F1TENTH simulator (https://github.com/f1tenth/f1tenth_gym_ros). With the simulator and keyboard teleop running simultaneously, start the node with `ros2 run safety_node safety_node`.


