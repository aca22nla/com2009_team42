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
        self.tb3_lidar = Tb3LaserScan()
        rospy.loginfo("tb3_lidar initialized successfully")

        self.min_front_distance = 0.33
        self.min_side_distance = 0.32

        self.front_dist = 0
        self.left_dist = 0
        self.right_dist = 0
       

        rospy.loginfo("The 'Maze Navigator' is active...")

    def calculate_angular_velocity(self):
        # Obstacle detected, calculate angular velocity to follow the gap
        angular_velocity = (self.front_dist - 0) / 20.0  # proportional control
        return angular_velocity

    
    def wall_follower(self):
        self.ang_vel = self.calculate_angular_velocity

        self.front_dist = self.tb3_lidar.front_dist
        self.left_dist = self.tb3_lidar.left_side_dist
        self.right_dist = self.tb3_lidar.right_side_dist
        

        print(f"front : {self.front_dist}")
        print(f"left : {self.left_dist}")
        print(f"right : {self.right_dist}")
       
        # Check if right is empty

        if self.right_dist > 0.5 :
            print(f"Right area empty, turning right to find wall")
            self.vel_controller.set_move_cmd(0.0, -1.0)
            self.vel_controller.publish()
            return
                
        # Check if there's an obstacle on the right side
        if self.right_dist > self.min_side_distance and self.right_dist <= 0.5:
            print(f"Finding the right wall")
            self.vel_controller.set_move_cmd(0.26, -0.12)
            self.vel_controller.publish()
            return

        # Follow the right wall
        if self.right_dist > 0.23 and self.right_dist <= self.min_side_distance:
            print(f"Following the right wall")
            self.vel_controller.set_move_cmd(0.27, 0.0)
            self.vel_controller.publish()
            return
        
        if self.front_dist <= self.min_front_distance : 
            if self.left_dist <= 0.3 and self.right_dist <= 0.3:
                print(f"Stuck in dead end. Reverse")
                self.vel_controller.set_move_cmd(-0.15, 0.0)
                self.vel_controller.publish()
                return
            elif self.left_dist <= 0.25:
                print(f"Obstacle ahead. Turning right to avoid")
                self.vel_controller.set_move_cmd(0.0, -0.7)
                self.vel_controller.publish()
                return
                
            else:
                print(f"Obstacle ahead. Turning left to find the wall")
                self.vel_controller.set_move_cmd(0.0, 1.0)
                self.vel_controller.publish()
                return
                # print(f"Turn slightly to the right")
                # self.vel_controller.set_move_cmd(0.0, -0.5 )
                # self.vel_controller.publish()

        

            
            
            
            
        # Move away from the right wall if too close
        if self.right_dist <= 0.23:
            print(f"Too close to the right wall. Moving away...")
            self.vel_controller.set_move_cmd(0.1, 0.11)
            self.vel_controller.publish()
            return
 
            
        if self.left_dist <= 0.24:
            print(f"Obstacles near left. Moving away...")
            self.vel_controller.set_move_cmd(0.1, -0.11)
            self.vel_controller.publish()
            return

            
        
        

        # # Check if there's an obstacle in front and the left side is clear
        # if self.front_dist <= self.min_front_distance and self.front_left < 0.2: 
        #     print(f"Obstacle ahead. Turning left to find the wall")
        #     self.vel_controller.set_move_cmd(0.0, 0.65)
        #     self.vel_controller.publish()
        #     return
        
        
               
        
        # If no other condition is met, stop the robot
        print("Stopping")
        self.vel_controller.set_move_cmd(0.0, 0.0)
        self.vel_controller.publish()

            
        
    def main(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            self.wall_follower()
            rate.sleep()
    
            

if __name__ == '__main__':
    node = MazeNav()
    node.main()
    rospy.spin()
