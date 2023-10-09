# Comprobo Warmup Project
## Reuben, Kate, and Rajiv
The first project for comprobo was an introduction to working in ROS with the neato robot platform. We created a number of behaviors for the robot. These include, teleop controll, square path navigation, wall following, object tracking and following, and obstcale avoidance.
### Robot Teleop
The teleop mode uses human imput to directly controll the robots motions. Specific keys were mapped to motion states. These keys allowed for motion forwards, motion backwards, turning left, turning right, and stoping.
### Driving in a Square
The goal of the square path navigation program was to drive the neato robot in a square pattern autonomously. In order to achieve this, we used a timer based system. The main loop runs in a loop that repeats for each side of the square. For each side, the neato moves forward for a predetrmined time, then turns 90 degrees. The turn function takes the number of degrees to turn and computes the wait time for the turn.

    def turn(self, degrees):
    
        """Turn to a specified angle
        Args:
            degrees (_type_): the turn angle in radians"""
    
        angular_vel = 0.3
        self.drive(linear=0 angular=angular_vel)
        sleep(degrees / angular_vel)
        self.drive(linear=0.0, angular=0.0)


The main flaw with this method is that there is no feedback from the neato sensor suite. The neato could run into an obstacle, or experience wheel slip on a low friction surface, and the program would continue executing with expectations for ideal conditions.
### Wall Following
The `WallFollowerNode` class is centered around making a robot navigate by following walls through the analysis of laser scan data. Upon class instantiation, several foundational parameters are set: `follow_offset` which determines how close the robot should be to the wall; `variance`, which is an allowable angle deviation to determine if the robot is parallel to the wall; `wall_distance` that indicates the proximity of the nearest wall; and `wall_angle`, which reflects the angle to the closest detected wall from the robot's perspective.

To process incoming sensor data, the node subscribes to the 'scan' topic, anticipating `LaserScan` messages. When these messages arrive, they are processed by the `process_laser` method. This method identifies the closest wall's distance and angle by analyzing valid range data points from the laser scan.

The `run_loop` method, triggered every 0.1 seconds, is where the robot's motion decisions are made. Depending on its current state, the robot can take actions like orienting towards the wall, approaching it, or following alongside it. If the robot identifies itself as parallel to the wall and at the desired offset, it proceeds straight. The code for checking this is as follows:

    closest_perpendicular = (self.wall_angle >= 180)*270 + (self.wall_angle < 180)*90
    if abs(self.wall_angle - closest_perpendicular) < self.variance:
        self.parallel = True
        print("\nPARALLEL")
    else:
        self.parallel = False
        print("\nNOT PARALLEL")

There are two main parts here. First, we find the closest perpendicular angle. The robot tries to determine which side (left or right) is closer to a wall. The line calculating `closest_perpendicular` does this by evaluating whether the angle to the nearest wall is greater than 180 degrees. If it is, the robot assumes the closest perpendicular angle is 270 degrees (right side); otherwise, it assumes 90 degrees (left side).

Next, we do the parallel check. The subsequent if condition checks if the robot's current orientation (`wall_angle`) is approximately perpendicular to the wall by comparing it to the `closest_perpendicular` value. It considers the robot parallel if the difference is within a predefined variance (angle tolerance). If parallel, the robot sets the `parallel` flag to True; otherwise, it sets it to False.

If not parallel to the wall, the robot adjusts its angular and linear velocities based on calculated proportional controls, aiming to correct its course or distance from the wall. Finally, motion commands are dispatched to the robot by publishing `Twist` messages to the 'cmd_vel' topic.
### Person Following
The PersonFollower class is designed to enable a robot to pursue a person using laser scan data from LIDAR. At its core, the class comprises several key functions:

- `__init__`: Initializes the node, setting up foundational parameters such as the desired following distance (`follow_offset`), angle variance for orientation (`variance`), and a threshold for second derivative analysis (`second_deriv_threshold`). It also establishes subscribers to laser scan data and publishers for robot velocity commands.

- `run_loop`: Activated every 0.1 seconds, this function examines the orientation of the robot relative to the detected person. If the robot isn't facing the person, it reorients. Once properly oriented, it advances towards the detected person.

- `process_laser`: A critical function that processes incoming LIDAR scans. It first cleans up the scan data, ensuring valid ranges. The function then calculates the first and second derivatives of the scan data. Using the second derivative, it identifies potential edges of the person in the environment. From this, it determines the angle (`person_angle`) and distance (`person_distance`) to the identified person.

- `filtered_minimum`: This function complements process_laser by identifying the nearest valid section of a laser scan that likely corresponds to a person. It utilizes the second derivative values and employs recursion to refine its search, excluding sections deemed invalid based on certain criteria.

- `control_heading`: This function is intended to return an angular velocity that ensures the robot's convergence on a goal angle relative to its current angle, providing a more refined control for orientation in the future.

In essence, the `PersonFollower` class offers a structured approach for person detection using laser scan data and facilitates the robot's movement in tracking and maintaining a specified distance from the detected individual.
### Obstalce Avoidance
The `ObstacleAvoidanceNode` class is structured to empower a robot to navigate its environment while evading obstacles using laser scan data from LIDAR. Upon initialization, the class initializes a flag (`front_obstacle`) to detect if there's an obstruction directly ahead and a variable (`turn_value`) to decide the robot's turn rate. Every 0.1 seconds, the `run_loop` function evaluates if an obstacle is in the robot's path. If detected, the robot halts its forward movement; otherwise, it proceeds forward. To discern the presence of obstacles, the `process_laser` function examines the front 30-degree range of the robot's field of view, setting the `front_obstacle` flag accordingly. Additionally, this function computes the mean distances on the right and left sides of the robot using the LIDAR data. The difference between these two means determines the `turn_value`, directing the robot to turn away from closer obstacles. Throughout, the node subscribes to laser scan data for real-time updates and publishes velocity commands to maneuver the robot based on detected obstacles.
### Combining Multiple Behaviors Using Finite-State Control
