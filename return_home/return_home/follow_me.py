import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy



import rclpy
from rclpy.node import Node
import time
import math
from sensor_msgs.msg import LaserScan
from return_home import return_home1

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos = QoSProfile(depth=10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile)
        self.subscription
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.not_moving_since = True
        self.first_move = True
        self.finished = False

    def listener_callback(self, msg):
        borne_max = 1
        borne_min = 0.2
        ouverture = 15
        laser_range = msg._ranges[-ouverture:] + msg._ranges[:ouverture]
        x_centre = (borne_max + borne_min) / 2
        y_centre = 0.0
        target = [
            (
                data * math.cos(math.radians(angle)),
                data * math.sin(math.radians(angle))
            )
            for angle, data in enumerate(laser_range, start=-ouverture)
            if borne_min < data < borne_max
        ]
        if target:
            avg_x = sum(x for x, _ in target) / len(target)
            avg_y = sum(y for _, y in target) / len(target)
        else:
            avg_x, avg_y = x_centre, y_centre

        delta_x = avg_x - x_centre
        delta_y = avg_y - y_centre
        self.move(delta_x, delta_y)
        if delta_x > 0.05 or delta_x < -0.05  or delta_y > 0.05 or delta_y < -0.05 :
            print('moving')

            self.not_moving_since = True
            
        else:
            print('not moving')
            if self.not_moving_since is True    :
                self.not_moving_since = time.time()
            if type(self.not_moving_since) is float:
                if time.time() - self.not_moving_since > 4:
                    # rclpy.shutdown()
                    # sys.exit()
                    self.finished = True


    def move(self, delta_x, delta_y):
        twist = Twist()
        k_lin = 1.0 
        k_ang = 5.0
        twist.linear.x = delta_x * k_lin
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = delta_y *k_ang

        self.pub.publish(twist)

def main(args=None):
    # rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    while not minimal_subscriber.finished:
        rclpy.spin_once(minimal_subscriber)

    minimal_subscriber.destroy_node()
    # rclpy.shutdown() #a verifier


if __name__ == '__main__':
    main()