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
import numpy as np
import math


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower_node')
        self.follow_offset = .3
        self.variance = 2         # degrees of acceptable variance
        self.second_deriv_threshold = .25
        
        self.person_distance = None 
        # angle between robot forward heading and person
        self.person_angle = None
        # is the robot parallel to the wall it wants to follow?
        # create subscriber to monitor bumps and publisher to send resultant vel
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_laser, 10)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.run_loop)               # call loop function every .1s
    
    def run_loop(self):
        """
        send velocity message based on bumper status
        """
        msg = Twist()
        # check to see if the laser function has been called/system is ready
        if not self.person_angle:
            return
        # Orient towards the person
        if not 180-self.variance<self.person_angle<180+self.variance:
            msg.angular.z = (self.person_angle-180)/50 if self.person_angle>180 else -(180-self.person_angle)/50
            # msg.angular.z = self.control_heading(self.wall_angle, 180)
            msg.linear.x = 0.0
            print("orienting to the person")
        # once you've oriented, you can travel towards the wall
        else:
            msg.angular.z = 0.0
            msg.linear.x = -0.1
            print("approaching the person")
        
        # LOGIC ABOVE THIS LINE WORKS   
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
        valid_ranges = [distance if \
                            msg.range_min <= distance <= msg.range_max \
                            else msg.range_max for distance in msg.ranges]
        # find the change between each point of the scan. Add additional point to correct for reduction of array by 1 element
        first_derivative = np.diff([valid_ranges[360]]+valid_ranges)
        second_derivative = np.diff([first_derivative[360]]+first_derivative)
        person_edges = np.array([angle for angle, value in enumerate(second_derivative) if value>self.second_deriv_threshold])
        person_edges = np.concatenate([[0], person_edges, [360]])
        print(person_edges)
        closest_point = valid_ranges.index(min(valid_ranges))
        self.person_distance = min(valid_ranges)    
        self.person_angle = self.filtered_minimum(np.asarray(valid_ranges), second_derivative, person_edges, .001)
        print(f"person detected at angle {self.person_angle}\nclosest wall at angle {closest_point}\n")
        
    def filtered_minimum(self, laser_scan, second_derivative, section_indexes, threshold):
        """
        find the minimum distance in a "valid" section of a list recursively while filtering sections without second derivative variation

        args:
            laser_scan (np array of floats): 360 element list with floats of laser scans and out of range scans as max scan distance
            second_derivative (list of floats): 360 element list with second derivatives of all degree measurements (between elements)
            section_indexes (list of ints): indexes of the largest second derivative points pre-filtered to divide data
            threshold (float): minimum threshold to apply to section's summed second derivatives

        returns:
            closest_valid_angle (int): index 0-360 of the closest valid point
            closest_valid_distance (float): distance value for the closest valid point in meters 
        """
        # find index of minimum laser scan value of array
        closest_point = np.argmin(laser_scan)
        # find section bookends before and after the closest point
        print(f"calculating section bookends with closest point {closest_point}")
        section_start = section_indexes[section_indexes>closest_point].min()
        section_end = section_indexes[section_indexes<closest_point].max()
        
        section = 
        section_weight = sum([abs(point) for point in second_derivative[section_start:section_end]])#/(.1*(section_end-section_start))
        if section_weight>threshold:
            print(f"accepted minimum between {section_start} and {section_end} at {closest_point}!!!!!!!!!")
            return closest_point
        else:
            print(f"rejected minimum between {section_start} and {section_end} with section weight {section_weight}")
            # replace laser scan values in the invalid section with the max range detected
            np.put(laser_scan, range(section_start, section_end+1), np.amax(laser_scan))
            # TODO: BROKEN HERE fix this returning None
            # recursively call function on the new filtered laser scan
            self.filtered_minimum(laser_scan, second_derivative, section_indexes, threshold)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
