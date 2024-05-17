#! /usr/bin/env python3
# search_server.py

import rospy
import waffle
import math
from testcolour_search import TestColourSearch

class RobotExplorer():
    
    def __init__(self):
        self.node = "explore_server"
        rospy.init_node(self.node)

        self.rate = rospy.Rate(10)
        self.target_colour = rospy.get_param("~target_colour", "")
        rospy.loginfo(f"Target Color: {self.target_colour}")
        self.colour_search = None
        if self.target_colour in ["yellow", "red", "green", "blue"]:
            self.colour_search = TestColourSearch(self.target_colour)
            print(f"Colour search initialized {self.target_colour}")
        
        # functions from the waffle.py module:
        self.motion = waffle.Motion(debug = True)
        self.pose = waffle.Pose(debug = True)
        self.lidar = waffle.Lidar(debug = True)
        rospy.loginfo("Lidar initialized successfully")


        self.min_front_distance = 0.4
        self.min_side_distance = 0.32

        self.state = "exploring"  # Initialize the state
        rospy.loginfo("The 'Explore Server' is active...")

    def calculate_angular_velocity(self, closest_object_location):
        orientation_difference = closest_object_location - self.pose.yaw
        # Normalize the orientation difference to the range [-pi, pi]
        orientation_difference = (orientation_difference + math.pi) % (2 * math.pi) - math.pi
        
        # Define the maximum allowed angular velocities
        pmax_angular_velocity = 1.4  # Maximum angular velocity in radians per second
        nmax_angular_velocity = -1.4  # Minimum angular velocity in radians per second
        
        # Calculate angular velocity based on the orientation difference
        angular_velocity = orientation_difference * 2
        
        # Limit angular velocity to the maximum allowed value
        if angular_velocity > pmax_angular_velocity:
            angular_velocity = pmax_angular_velocity
        elif angular_velocity < nmax_angular_velocity:
            angular_velocity = nmax_angular_velocity

        return angular_velocity 
    def explore(self):
        self.motion.set_velocity(linear=0.2, angular=0.0)
        self.motion.publish_velocity()

    def avoid_obstacle(self):
        #object too near
        

        if self.closest_object <= self.min_front_distance and self.right_obstacle <= self.min_side_distance:
            self.motion.set_velocity(linear=0.0, angular=1.6)
            self.motion.publish_velocity()
            print("Right corner. Turn left")
        elif self.closest_object <= self.min_front_distance and self.left_obstacle <= self.min_side_distance:
            self.motion.set_velocity(linear=0.0, angular=-1.6)
            self.motion.publish_velocity()
            print("Left corner. Turn right")
        elif self.closest_object <= self.min_front_distance:
            angular_velocity = self.calculate_angular_velocity(self.closest_object)
            self.motion.set_velocity(linear=0.0, angular=angular_velocity)
            self.motion.publish_velocity()
        elif self.right_obstacle <= self.min_side_distance:
            self.motion.set_velocity(linear=0.1, angular=0.5)
            self.motion.publish_velocity()
            print("Approaching right obstacle")
        elif self.left_obstacle <= self.min_side_distance:
            self.motion.set_velocity(linear=0.1, angular=-0.5)
            self.motion.publish_velocity()
            print("Approaching left obstacle")
        if self.closest_object <= 0.25 or self.left_obstacle <= 0.25 or self.right_obstacle <= 0.25:
            self.motion.set_velocity(linear=-0.1, angular=0.0)
            self.motion.publish_velocity()
            print("Too near. Move backwards")


    

    def check_obstacles(self):
        self.closest_object = min(self.lidar.subsets.frontArray)
<<<<<<< HEAD
        self.right_obstacle = min(self.lidar.right_side)
        self.left_obstacle = min(self.lidar.left_side)
=======
        self.right_obstacle = min(self.lidar.right_obstacle)
        self.left_obstacle = min(self.lidar.left_obstacle)
>>>>>>> 42084e245c3fd800e4578abd4fb3a3056e4f2a08

    def detect_beacon(self):
        return self.colour_search.is_beacon_detected()
    
    
    def main(self, img_data):

        while not rospy.is_shutdown():
            self.check_obstacles()

            

            if self.target_colour and self.colour_search:

                if self.closest_object <= self.min_front_distance or self.right_obstacle <= self.min_side_distance or self.left_obstacle <= self.min_side_distance:
                    rospy.loginfo("Obstacle detected. Avoiding obstacle")
                    self.state = "avoiding_obstacle"
                    
                elif self.detect_beacon():
                    rospy.loginfo("Beacon detected. Approaching beacon")
                    self.state = "approaching_beacon"
                else:
                    self.state = "exploring"

                if self.state == "avoiding_obstacle":
                    self.avoid_obstacle()
                    rospy.sleep(1)  # Allow the robot to turn for 1 second
                    self.state = "exploring"
                elif self.state == "approaching_beacon":
                    self.approach_beacon()
                elif self.state == "exploring":
                    self.explore()

                
            

                

            else: 
                if self.closest_object <= self.min_front_distance or self.right_obstacle <= self.min_side_distance or self.left_obstacle <= self.min_side_distance:
                    rospy.loginfo("Obstacle detected. Avoiding obstacle")
                    self.state = "avoiding_obstacle"
                    
                else:
                    self.state = "exploring"

                if self.state == "avoiding_obstacle":
                    self.avoid_obstacle()
                    rospy.sleep(1)  # Allow the robot to turn for 1 second
                    self.state = "exploring"
                
                elif self.state == "exploring":
                    self.explore()

            img_data = self.colour_search.get_latest_img_data()
            if img_data is None:
                rospy.logerr("No image data available")



            

            self.rate.sleep()

    def approach_beacon(self):
        self.motion.set_velocity(linear=0.2, angular=0.0)
        self.motion.publish_velocity()


    

if __name__ == '__main__':
    node = RobotExplorer()
    img_data = node.colour_search.get_latest_img_data()
    node.main(img_data)
    rospy.spin()

#     def main(self):

#         while not rospy.is_shutdown():

#             # Get the robot's current odometry
#             posx0 = self.pose.posx
#             posy0 = self.pose.posy

#             # Check LiDAR data
#             self.closest_object = min(self.lidar.subsets.frontArray)
#             self.right_obstacle = min(self.lidar.subsets.l1, self.lidar.subsets.l2)
#             self.left_obstacle = min(self.lidar.subsets.r1, self.lidar.subsets.r2)

#             if self.target_colour and self.colour_search:

#             #check if there is object detected
#                 if self.closest_object <= self.min_front_distance:
#                     rospy.loginfo("Obstacle detected. Avoiding obstacle")

#                     # Calculate angular velocity to avoid the obstacle
#                     angular_velocity = self.calculate_angular_velocity(self.closest_object)
#                     self.motion.set_velocity(linear=0.0, angular=angular_velocity)
#                     self.motion.publish_velocity()

#                     rospy.sleep(1)  # Allow the robot to turn for 1 second

#                 else:
#                     # No obstacle detected, continue exploration
#                     self.motion.set_velocity(linear=0.2, angular=0.0)
#                     self.motion.publish_velocity()

#                 img_data = self.colour_search.get_latest_img_data()
#                 self.colour_search.camera_callback(img_data) # To perform colour detection

#             else:

#                 if self.closest_object <= self.min_front_distance:
#                     rospy.loginfo("Obstacle detected. Avoiding obstacle")

#                     # Calculate angular velocity to avoid the obstacle
#                     angular_velocity = self.calculate_angular_velocity(self.closest_object)
#                     self.motion.set_velocity(linear=0.0, angular=angular_velocity)
#                     self.motion.publish_velocity()

#                     rospy.sleep(1)  # Allow the robot to turn for 1 second
#                 else:
#                     # No obstacle detected, continue exploration
#                     self.motion.set_velocity(linear=0.2, angular=0.0)
#                     self.motion.publish_velocity()

#             posx1 = self.pose.posx
#             posy1 = self.pose.posy
#             # determine how far the robot has travelled so far:
#             self.distance = math.sqrt(pow(posx1 - posx0, 2) + pow(posy1 - posy0, 2))

#             posx0, posy0 = posx1, posy1

#             # if img_data is None:
#             #     rospy.logerr("No image data available")   
#             # else:
#             #     img_data
                 
#             self.rate.sleep()


# if __name__ == '__main__':
#     node = RobotExplorer()
#     node.main()
#     rospy.spin()
