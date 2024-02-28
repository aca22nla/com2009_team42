#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Circle():

    def __init__(self):
        self.node_name = "circle_mover"

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10) 

        self.sub = rospy.Subscriber("odom", Odometry, self.callback)

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook)

        self.counter = 11

        self.direction = 1  # Direction of rotation: 1 for counterclockwise, -1 for clockwise
        self.circle_count = 0  # Count of completed circles
        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation 
        
        initial_pos_x = None
        initial_pos_y = None
        initial_pos_z = None
        pos_x = position.x #forward backwards
        pos_y = position.y #right left
        pos_z = position.z #up down
        if initial_pos_x == None:
            initial_pos_x = pos_x
        if initial_pos_y == None:
            initial_pos_y = pos_y
        if initial_pos_z == None:
            initial_pos_z = pos_z

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        
        if self.counter > 10:
            self.counter = 0
            print(f"x={pos_x-initial_pos_y:.2f} [m], y={pos_y-initial_pos_y:.2f} [m], yaw={yaw:.1f} [degrees].")
        else:
            self.counter += 1

    def shutdownhook(self):
        stop_cmd = Twist()
        self.pub.publish(stop_cmd)
        self.ctrl_c = True

    def main(self):
        angular_speed = 1.0  # Angular speed in rad/s
        duration = 6.28 / angular_speed  # Time to complete one circle (2*pi / angular_speed)

        while not self.ctrl_c and self.circle_count < 2:  # Stop after completing one counterclockwise and one clockwise circle
            start_time = rospy.Time.now().to_sec()  # Get the current time
            while not self.ctrl_c:
                current_time = rospy.Time.now().to_sec()  # Get the current time
                elapsed_time = current_time - start_time  # Calculate elapsed time

                # Check if one full circle (360°) is completed
                if elapsed_time >= duration:
                    rospy.loginfo(f"One full circle completed in direction {'counterclockwise' if self.direction == 1 else 'clockwise'}.")
                    self.circle_count += 1  # Increment circle count
                    self.direction *= -1  # Reverse direction for the next circle
                    break  # Break out of the loop to switch direction

                # Publish movement commands
                move_cmd = Twist()
                move_cmd.linear.x = 0.5  # Linear velocity
                move_cmd.angular.z = angular_speed * self.direction  # Angular velocity with direction
                self.pub.publish(move_cmd)  # Publish movement command
                self.rate.sleep()

if __name__ == '__main__':
    circle = Circle()
    circle.main() 
