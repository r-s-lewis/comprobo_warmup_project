"""
Drive Square
--------
Drive in the shape of a square
"""
import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class DrawSquare(Node):
    """Pilot the neato in a square.
    """

    def __init__(self):
        super().__init__('draw_square')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

    def run_loop(self):
        """Draws a square by moving forward and turning four times.
        """
        self.drive(0.0, 0.0)
        sleep(1)
        for _ in range(4):
                print("driving forward")
                self.drive_forward(0.5)
                print("turning")
                self.turn(math.pi/2)
        print("done")

    def drive(self, linear, angular):
        """Drive the neato

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """        
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def turn(self, degrees):
        """Turn to a specified angle

        Args:
            degrees (_type_): the turn angle in radians
        """    

        angular_vel = 0.3
        self.drive(linear=0.0, angular=angular_vel)
        sleep(degrees / angular_vel)
        self.drive(linear=0.0, angular=0.0)

    def drive_forward(self, distance):
        """Drive in a straight line

        Args:
            distance (_type_): the distance to drive forward.  Only positive
            values are supported.
        """
        forward_vel = 0.1
        self.drive(linear=forward_vel, angular=0.0)
        sleep(distance / forward_vel)
        self.drive(linear=0.0, angular=0.0)

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()