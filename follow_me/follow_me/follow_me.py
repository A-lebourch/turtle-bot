# #!/usr/bin/env python
# #
# # Copyright (c) 2011, Willow Garage, Inc.
# # All rights reserved.
# #
# # Software License Agreement (BSD License 2.0)
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions
# # are met:
# #
# #  * Redistributions of source code must retain the above copyright
# #    notice, this list of conditions and the following disclaimer.
# #  * Redistributions in binary form must reproduce the above
# #    copyright notice, this list of conditions and the following
# #    disclaimer in the documentation and/or other materials provided
# #    with the distribution.
# #  * Neither the name of {copyright_holder} nor the names of its
# #    contributors may be used to endorse or promote products derived
# #    from this software without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.
# #
# # Author: Darby Lim

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile



import rclpy
from rclpy.node import Node
import time
import math
from sensor_msgs.msg import LaserScan


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)

    def listener_callback(self, msg):
        # self.get_logger().info("ranges")
        target = []
        x_list = []
        y_list = []
        borne_max = 2
        borne_min = 0.5
        laser_range = []
        ouverture = 20
        for t in range(ouverture):
            laser_range.append(msg._ranges[ouverture - t])
        for t in range(ouverture):
            laser_range.append(msg._ranges[-t])
        
        angle = -ouverture
        x_centre = (borne_max + borne_min) /2
        y_centre = 0.0
        for data in laser_range:
            if data > borne_min and data < borne_max:
                x = data * math.cos(math.radians(angle))
                y = data * math.sin(math.radians(angle))
                target.append([x, y])
                x_list.append(x)
                y_list.append(y)
                # print(data)
            angle += 1
        if x_list != [] and y_list != []:
            avg = [sum(x_list) / len(x_list), sum(y_list) / len(y_list)]
            print(avg)
        else: 
            avg = [x_centre, y_centre]

        delta_x = avg[0] - x_centre
        delta_y = y_centre - avg[1]

        self.move(delta_x, delta_y)

    def move(self, delta_x, delta_y):
        twist = Twist()
        k_lin = 0.5
        k_ang = 3.0
        twist.linear.x = delta_x * k_lin
        twist.linear.y = 0.0
        twist.linear.z = 0.0


        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = delta_y *k_ang

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()