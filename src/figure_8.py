#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from math import pi

import time

class Circle():

    def __init__(self):
        #--node info
        self.node_name = "figure_of_8"
        self.topic_name = "/cmd_vel"

        #--initialisation and functions
        rospy.init_node(self.node_name, anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)#sends info (instructions, vel, etc) to bot
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)#gets info (odometry) from bot
        self.rate = rospy.Rate(10) 
        
        self.counter = 0 # for odometry data
        self.initial_pos_x = None # starting pos of bot 
        self.initial_pos_y = None
        self.initial_pos_z = None

        self.RADIUS = 0.5  # Radius of the circle (meters)
        self.CIRCUMFERENCE = 2 * pi * self.RADIUS
        self.time_for_loop = 30
        self.linear_vel = self.CIRCUMFERENCE / self.time_for_loop
        self.angular_vel = self.linear_vel / self.RADIUS


        self.elapsed_time = 0

        #shutdown
        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation 
        
        pos_x = position.x #forward backwards
        pos_y = position.y #right left
        pos_z = position.z #up down
        if self.initial_pos_x == None:
            self.initial_pos_x = pos_x
        if self.initial_pos_y == None:
            self.initial_pos_y = pos_y
        if self.initial_pos_z == None:
            self.initial_pos_z = pos_z

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        
        if self.counter <= 0:
            self.counter = 5
            print(f"x={pos_x-self.initial_pos_y:.2f} [m], y={pos_y-self.initial_pos_y:.2f} [m], yaw={yaw:.1f} [degrees].")
        else:
            self.counter -= 1

    def shutdownhook(self): # stops and exits on shutdown
        
        self.stop_moving()

        self.ctrl_c = True

    def stop_moving(self):
        stop_cmd = Twist()

        stop_cmd.linear.x = 0.0 # m/s
        stop_cmd.angular.z = 0 # rad/s
        self.pub.publish(stop_cmd)

    def loop(self, loop_number):
        move_cmd = Twist()

        n = 1 if loop_number == 1 else -1

        move_cmd.linear.x = self.linear_vel # m/s
        move_cmd.angular.z = n*(move_cmd.linear.x / self.RADIUS) # rad/s
        self.pub.publish(move_cmd)  # Publish movement command
        self.rate.sleep()

    def main(self):

        start_time = rospy.Time.now().to_sec()
        elapsed_time = self.elapsed_time

        while elapsed_time <= (self.time_for_loop + 0.5) and not self.ctrl_c:
            current_time = rospy.Time.now().to_sec()  # Get the current time
            elapsed_time = current_time - start_time  # Get the elapsed time
            self.loop(1)
        print("########_FIRST LOOP DONE_########")
        
        while elapsed_time > (self.time_for_loop + 0.5) and elapsed_time <= (2 * (self.time_for_loop + 0.5)) and not self.ctrl_c:
            current_time = rospy.Time.now().to_sec()  # Get the current time
            elapsed_time = current_time - start_time
            self.loop(2)
        print("########_SECOND LOOP DONE_########")

        self.shutdownhook()
        

if __name__ == '__main__':
    circle = Circle()
    circle.main()