"""
node to handle stopping robot when it experiences a bump
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import statistics


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.front_obstacle = False
        self.turn_value = 0.0
        self.create_timer(0.1, self.run_loop)               # call loop function every .1s
        # create subscriber to monitor bumps and publisher to send resultant vel
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_laser, 10)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def run_loop(self):
        """
        send velocity message based on bumper status
        """
        msg = Twist()
        if self.front_obstacle:
            msg.linear.x = 0.0
        else:
            msg.linear.x = 0.1
        msg.angular.z = self.turn_value
        self.vel_publisher.publish(msg)
        
    def process_laser(self, msg:LaserScan):
        """
        process bump message and 
        """
        msg.range_min
        front_distance = msg.ranges[0]
        valid_ranges = [distance if \
                            msg.range_min <= distance <= msg.range_max \
                            else None for distance in msg.ranges]
        if valid_ranges[0] is not None and valid_ranges[0] < .5:
            self.front_obstacle = True
        else:
            self.front_obstacle = False
        
        right_sensor_values = statistics.mean([distance for distance in valid_ranges[0:90] if distance is not None])  
        left_sensor_values = statistics.mean([distance for distance in valid_ranges[270:360] if distance is not None])  
        self.turn_value = right_sensor_values - left_sensor_values
        print(self.front_obstacle, self.turn_value)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
