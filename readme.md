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
### Person Following
### Obstalce Avoidance
### Combining Multiple Behaviors Using Finite-State Control
