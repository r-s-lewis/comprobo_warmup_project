"""
node to handle stopping robot when it experiences a bump

takes closest point in laser scan, 
if it is closer than the following distance (.5m) turns away and travels away until at .5m
if it is farther than the following distance, turns towards the wall and travels until it reaches following distance
once at following distance, 
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import statistics
import math


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.follow_offset = .3
        self.variance = 2         # degrees of acceptable variance
        
        # is the robot close enough to the wall it wants to follow?
        self.wall_distance = None # TODO: remove this junk value
        # angle between robot forward heading and wall
        self.wall_angle = None
        # is the robot parallel to the wall it wants to follow?
        self.parallel = False
        # this is a bad idea
        self.test_bullshit = 0
        # create subscriber to monitor bumps and publisher to send resultant vel
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_laser, 10)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.run_loop)               # call loop function every .1s
    
    def run_loop(self):
        """
        send velocity message based on bumper status
        """
        msg = Twist()
        if not self.wall_distance:
            return
        # If you're too far from the wall, orient to face the wall and travel towards it
        if self.wall_distance > self.follow_offset and not self.test_bullshit:
            # if you're not facing away from the wall, orient to face away (180)
            if not 180-self.variance<self.wall_angle<180+self.variance:
                msg.angular.z = (self.wall_angle-180)/50 if self.wall_angle>180 else -(180-self.wall_angle)/50
                # msg.angular.z = self.control_heading(self.wall_angle, 180)
                msg.linear.x = 0.0
                print("orienting to the wall")
            # once you've oriented, you can travel towards the wall
            else:
                msg.angular.z = 0.0
                msg.linear.x = -0.1
                print("approaching the wall")
        
        # LOGIC ABOVE THIS LINE WORKS
        elif self.test_bullshit == 2:
            ## lets see what happens here
            msg.angular.z = 0.0
            msg.linear.x = 0.1

            

        else:
            self.test_bullshit = 1
            # identify the closest perpendicular angle (270 or 90)
            closest_perpendicular = (self.wall_angle >= 180)*270 + (self.wall_angle < 180)*90
            msg.angular.z = .5 if closest_perpendicular == 90 else -.5
            # command the robot to make the wall angle perpendicular to the bot TODO: debug this!
            # msg.angular.z = (self.wall_angle-closest_perpendicular)/50 if self.wall_angle>closest_perpendicular else (closest_perpendicular-self.wall_angle)/50
            # msg.angular.z = 0.0 # this is an intentional overwrite bc previous line is broken
            # msg.angular.z = self.control_heading(self.wall_angle, closest_perpendicular)
            # msg.linear.x = 0.1
            if abs(self.wall_angle - 90) <= 10 or abs(self.wall_angle - 270) <= 10:
                self.test_bullshit = 2
            print(f"closest perpendicular {closest_perpendicular}")
        self.vel_publisher.publish(msg)
        # print(f"sending velocity {msg.linear.x}\nrotation {msg.angular.z}")

    def control_heading(self, current_angle, goal_angle):
        """
        TODO: replace the angular velocity commands with control_heading calls
        WARNING currently not used or tested but future implement

        returns an angular velocity that will converge on a goal angle based on the current angle

        args:
            current_angle (int): body-frame degrees 0-360 with 0 and 360 indicating forward, 90 indicating left 270 indicating right
            goal_angle (int): desired heading in degrees of the sensor measurement currently at current_angle
        """
        scale = 100
        angular_vel = (current_angle-goal_angle)/scale if current_angle>goal_angle else -(goal_angle-current_angle)/scale
        return angular_vel
    def process_laser(self, msg:LaserScan):
        """
        process bump message and 
        """
        front_distance = msg.ranges[0]
        valid_ranges = [distance if \
                            msg.range_min <= distance <= msg.range_max \
                            else 2.0 for distance in msg.ranges]
        # angle in degrees of the closest wall (detected point) to the neato
        self.wall_angle = valid_ranges.index(min(valid_ranges))
        self.wall_distance = min(valid_ranges)
        print(f"closest wall at angle {self.wall_angle}\nclosest wall at distance {self.wall_distance}\n")
        
        # right_sensor_values = statistics.mean([distance for distance in valid_ranges[0:90] if distance is not None])  
        # left_sensor_values = statistics.mean([distance for distance in valid_ranges[270:360] if distance is not None])  
        # self.turn_value = right_sensor_values - left_sensor_values
        # print(self.wall_distance, self.turn_value)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
