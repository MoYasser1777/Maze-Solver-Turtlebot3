#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import math
import time
import matplotlib.pyplot as plt
import numpy as np


DISTANCE_TO_ROTATE = 0.8
TURN_FIX_FACTOR = 3.0
RIGHT = -1
LEFT = 1

class WallFollower():
    def __init__(self, speed, distance_to_wall, side):

        self.x_history = []
        self.y_history = []

        self.x_history_odom = []
        self.y_history_odom = []

        self.total_distance = 0.0
        self.last_position = None

        self.speed = speed
        self.distance_wall = distance_to_wall
        self.wall_lead = 1.0

        self.side = LEFT if side == 'left' else RIGHT
        self.ang_vel = 0

        self.wide_center = 0

        rospy.init_node('maze_solver')

        self.rate = rospy.Rate(10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        print(f'wall following from {side} side')
        rospy.sleep(1)


    def update_velocity(self, linear_vel, angular_vel):

        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)


    def scan_callback(self, msg):
        # Get the maximum range of the laser scanner
        sensor_max_range = msg.range_max

        # Directly in front of the robot
        center = min(min(msg.ranges[0:5] + msg.ranges[-5:]), sensor_max_range)

        # Left regions
        left_1 = min(min(msg.ranges[11:20]), sensor_max_range)  # Slightly to the left
        left_2 = min(min(msg.ranges[64:73]), sensor_max_range)  # Even farther left
        left = min(min(msg.ranges[86:95]), sensor_max_range)    # Furthest left region considered

        # Right regions
        right_1 = min(min(msg.ranges[341:350]), sensor_max_range)  # Slightly to the right
        right_2 = min(min(msg.ranges[289:298]), sensor_max_range)  # Even farther right
        right = min(min(msg.ranges[266:275]), sensor_max_range)    # Furthest right region considered

        # Wider and narrower central ranges
        self.wide_center = min(min(msg.ranges[0:40] + msg.ranges[-40:]), 10)  # Wider central range
        narrow_center = min(min(msg.ranges[0:20] + msg.ranges[-20:]), 10)  # Narrower central range


        #------------------------------------the robot not following the wal---------------------------------------------------------------#

        # Determine wall distance based on the side the robot is following
        wall_distance = right if self.side == RIGHT else left
        # Calculate x and y orientation based on the laser data for angular adjustment
        orientation_x = (right_2 if self.side == RIGHT else left_2) * math.sin(math.radians(22.5))
        orientation_y = (right_2 if self.side == RIGHT else left_2) * math.cos(math.radians(22.5))

        # Check if the robot is heading directly into a wall
        if wall_distance >= self.distance_wall * 2 and center < sensor_max_range:
            # Turn by 45 degree to avoid collision
            self.ang_vel = -math.pi / 4 * self.side

        else:
            # Determine the minimum distance around the robot's side
            if(self.side == RIGHT):
                scan_around = min([center, right_1 + (sensor_max_range - right_2)])
            else:
                scan_around = min([center, left_1 + (sensor_max_range - left_2)])   
           # Adjust turning factor based on proximity to obstacles
            turn_fix_factor = (0 if scan_around >= DISTANCE_TO_ROTATE else 1.3 - scan_around) *TURN_FIX_FACTOR

            # Calculate angular velocity to maintain wall-following behavior
            # The alpha angle accounts for the difference between desired and actual orientation
            alpha = math.atan2(orientation_y - self.distance_wall,orientation_x - wall_distance + self.wall_lead) - turn_fix_factor
            
            # Adjust angular velocity direction based on the chosen side
            self.ang_vel = self.side * alpha


    def odom_callback(self, msg):
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self.last_position is not None:
            # Compute distance between last and current position
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            self.total_distance += math.sqrt(dx**2 + dy**2)

        # Update the last position
        self.last_position = current_position
        # Store positions for history
        self.x_history_odom.append(current_position[0])
        self.y_history_odom.append(current_position[1])
        self.x_history.append(current_position[0])
        self.y_history.append(current_position[1])

    def stop(self):

        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0

        self.cmd_vel_pub.publish(msg)
        self.rate.sleep()


    def reset(self):
        service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        service()
        self.x_history = []
        self.y_history = []

    def plot_path(self):

        if len(self.x_history) - len(self.y_history) != 0:

            for i in range(0,len(self.x_history) - len(self.y_history)):
                self.y_history.append(None)

        plt.plot(self.y_history, self.x_history)
        plt.title("Wall-Following Robot Path")
        plt.axis([10, -10, -10, 10])
        plt.show()  
    
    def solve(self):

        try:
            solved = False

            t0 = time.time()
            # Main loop to move the robot until the exit is reached or the node shuts down
            while self.wide_center < 10 and not rospy.is_shutdown():
                self.update_velocity(self.speed, self.ang_vel)
                self.rate.sleep()
            t1 = time.time()
            self.stop()
            self.plot_path()
            rospy.sleep(5.0)
            self.reset()
            solved = True

            return self.total_distance , t1-t0, solved

        except rospy.ROSInterruptException:
            pass
