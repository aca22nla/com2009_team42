#! /usr/bin/env python3
# search_server.py

import rospy

from tuos_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
import math 

class MazeNav():

    def __init__(self):
        self.consecutive_obstacles = 0
        self.node_name = "maze_navigator"
        rospy.init_node(self.node_name, anonymous=True)

        # functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
        rospy.loginfo("tb3_lidar initialized successfully")


        self.min_front_distance = 0.3
        self.min_side_distance = 0.33
        self.dynamic_threshold_scale = 0.5

        self.front_dist = 0
        self.left_dist = 0
        self.right_dist = 0
        self.front_right = 0
        self.front_left = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("The 'Maze Navigator' is active...")

    def shutdownhook(self):
        self.vel_controller.publish
        self.ctrl_c = True

    # def calculate_angular_velocity(self):
    #     # Calculate error (difference between desired and actual distance)
    #     error = self.min_safe_distance - self.front_dist

    #     # Proportional control
    #     Kp = 1.0  # Proportional gain
    #     angular_velocity = error * Kp

    #     # Limit angular velocity to a reasonable range
    #     max_angular_velocity = 1.2
    #     angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)

    #     return angular_velocity
    
    def wall_follower(self):
        # angular_velocity = self.calculate_angular_velocity()

        self.front_dist = self.tb3_lidar.front_dist
        self.left_dist = self.tb3_lidar.left_side_dist
        self.right_dist = self.tb3_lidar.right_side_dist
        self.front_right = self.tb3_lidar.front_right_side_dist
        self.front_left = self.tb3_lidar.front_left_side_dist

        print(f"front : {self.front_dist}")
        print(f"left : {self.left_dist}")
        print(f"right : {self.right_dist}")
        print(f"front right : {self.front_right}")
        print(f"front left : {self.front_left}")

        # Check if right is empty
        if self.right_dist > 0.5 :
            print(f"Right area empty, turning right to find wall")
            self.vel_controller.set_move_cmd(0.0, -1.0)
            self.vel_controller.publish()
            return

        # # Check if there's an obstacle in front and the right side is clear
        # if self.front_dist <= self.min_front_distance and self.right_dist > self.min_side_distance:
        #     print(f"Obstacle ahead. Turning right to find the wall")
        #     self.vel_controller.set_move_cmd(0.0, -0.5)
        #     self.vel_controller.publish()
        #     return

        # Check if there's an obstacle in front and the left side is clear
        if self.front_dist <= self.min_front_distance: 
            if self.front_left <= 0.2:
                print(f"Obstacles near front left. Moving away...")
                self.vel_controller.set_move_cmd(0.0, -1.0)
                self.vel_controller.publish()
                return
            else:
                print(f"Obstacle ahead. Turning left to find the wall")
                self.vel_controller.set_move_cmd(0.0, 1.0)
                self.vel_controller.publish()
                return

        # Check if there's an obstacle on the right side
        if self.right_dist > self.min_side_distance and self.right_dist <= 0.5:
            print(f"Finding the right wall")
            self.vel_controller.set_move_cmd(0.15, -0.1)
            self.vel_controller.publish()
            return

        # Follow the right wall
        if self.right_dist > 0.2 and self.right_dist <= self.min_side_distance:
            print(f"Following the right wall")
            self.vel_controller.set_move_cmd(0.2, 0.0)
            self.vel_controller.publish()
            return

        # Move away from the right wall if too close
        if self.right_dist <= 0.2 or self.front_right < 0.18:
            print(f"Too close to the right wall. Moving away...")
            self.vel_controller.set_move_cmd(0.1, 0.2)
            self.vel_controller.publish()
            return
        
        


        # If no other condition is met, stop the robot
        print("Stopping")
        self.vel_controller.set_move_cmd(0.0, 0.0)
        self.vel_controller.publish()

        # if self.right_dist > 0.5 :
        #     print(f"Right area empty, turning right to find wall")
        #     self.vel_controller.set_move_cmd(0.0, -1.0)
        #     self.vel_controller.publish()
        #     rospy.sleep(1)

        # elif self.front_dist <= self.min_front_distance:
        #     print(f"Obstacle ahead. Turn left")
        #     self.vel_controller.set_move_cmd(0.0, 1.0)
        #     self.vel_controller.publish()
        #     rospy.sleep(1)

        # elif self.right_dist > self.min_side_distance and self.right_dist <= 0.5:
        #     print(f"Finding right wall")
        #     self.vel_controller.set_move_cmd(0.15, -0.1)
        #     self.vel_controller.publish()

        # elif self.right_dist > 0.21 and self.right_dist <= self.min_side_distance:
        #     print(f"Move straight ahead.. Following right wall")
        #     self.vel_controller.set_move_cmd(0.2, 0)
        #     self.vel_controller.publish()
        
        # elif self.right_dist <= 0.21:
        #     print(f"Too close to right wall. Moving away...")
        #     self.vel_controller.set_move_cmd(0.0, 0.1)
        #     self.vel_controller.publish()

         
        # else:
        #     print("Stop")
        #     self.vel_controller.set_move_cmd(0.0, 0.0)
        #     self.vel_controller.publish()



            
        
    def main(self):

        rate = rospy.Rate(10)
        # angular_velocity = self.calculate_angular_velocity()

        while not rospy.is_shutdown():

            self.wall_follower()
            # print(f"Front distance: {self.front_dist}")
            # print(f"right distance: {self.right_dist}")
            # print(f"left distance: {self.right_dist}")
            
            # print(f"front right distance: {self.front_right}")
            # print(f"front left distance: {self.front_left}")
            # right corner, turn left
            
        

            # elif self.right_dist < self.min_side_distance :
            #     print("Too near! Move away from the wall")
            #     self.vel_controller.set_move_cmd(-0.1, 0.1)
            #     self.vel_controller.publish()
            

                # elif self.right_dist <= 0.15 or self.front_dist <= 0.15:
                #     print(f"Too close to wall. Move backwards")
                #     self.vel_controller.set_move_cmd(-0.1, 0.0)
                #     self.vel_controller.publish()
                #     rospy.sleep(1)

            # elif self.front_dist <= self.min_safe_distance and self.left_dist > self.min_side_distance:
            #     rospy.loginfo(f"Wall ahead in {self.front_dist}. Follow right wall. Turn left")
            #     self.vel_controller.set_move_cmd(0.0, -angular_velocity)
            #     self.vel_controller.publish()
            #     rospy.sleep(2)  # Turn for 1 second


        
                # elif self.left <= self.min_safe_distance:
                #     rospy.loginfo(f"Obstacle detected in front at {self.front_dist}. Avoiding obstacle")
                #     self.vel_controller.stop
                #     self.vel_controller.set_move_cmd(0.0, angular_velocity)
                #     self.vel_controller.publish()
                #     rospy.sleep(1)  # Turn for 1 second

            # elif self.right_dist <= self.min_side_distance:
            #     rospy.loginfo("Right corner. Turn left")
            #     self.vel_controller.set_move_cmd(0.0, -angular_velocity)
            #     self.vel_controller.publish()
            #     rospy.sleep(1)  # Turn for 1 second
                

            rate.sleep()
    



            

if __name__ == '__main__':
    node = MazeNav()
    node.main()
    rospy.spin()
