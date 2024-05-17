#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
from pathlib import Path

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import NumPy
import numpy as np

# Import some other modules from within this package
from tb3 import Tb3Move

from std_msgs.msg import Float32

class TestColourSearch():

    def __init__(self, target_colour):
        # node_name = "turn_and_face"
        # rospy.init_node(node_name)

        # rospy.init_node('colour_search', anonymous=True) 

        self.target_colour = target_colour
        self.target_hsv_lower, self.target_hsv_upper = self.get_hsv_threshold(target_colour)
        
        if self.target_colour not in ["yellow", "red", "green", "blue"]:
            rospy.logerr("Invalid target colour specified! Must be one of: yellow, red, green, blue.")
            rospy.signal_shutdown("Invalid target colour specified")

        rospy.loginfo(f"TASK 4 BEACON: The target is {self.target_colour}.")

        self.camera_subscriber = rospy.Subscriber(
            "/camera/color/image_raw",
            Image, self.camera_callback
        )
        self.cvbridge_interface = CvBridge()

        # Get the path to the package directory
        package_path = Path(__file__).parent.parent
        self.snaps_folder = package_path.joinpath("snaps")
        self.snaps_folder.mkdir(parents=True, exist_ok=True)

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(10)
        
        self.m00 = 0
        self.m00_min = 10000

        self.centroid_publisher = rospy.Publisher('/beacon_centroid', Float32, queue_size=10)
        self.latest_img_data = None
        self.beacon_detected = False

    def get_hsv_threshold(self, target_colour):
        if target_colour == "blue":
            # return ((115, 224, 100), (130, 255, 255))
            return ((102.027, 221.155, 100), (105.186, 255, 255))
        elif target_colour == "green":
            # return ((50, 150, 100), (65, 255, 255))
            return ((82.7, 196, 100), (89.5, 254, 255))
        elif target_colour == "red":
            # return ((0, 185, 100), (10, 255, 255))
            return ((2.032, 197.604, 100), (4.029, 252.456, 255))
        elif target_colour == "yellow":
            # return ((50, 150, 100), (65, 255, 255))
            return ((24.8727, 201.634, 100), (26.031, 250.234, 255))
        else:
            rospy.logerr(f"Invalid target colour: {target_colour}")
            return None, None
        
    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def save_image(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
            image_path = self.snaps_folder.joinpath("task4_beacon.jpg")
            cv2.imwrite(str(image_path), cv_img)
            rospy.loginfo(f"Beacon image saved successfully at {image_path}.")
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def camera_callback(self, img_data):
        if img_data is None:
            return
        
        # Check if the target_colour parameter is specified 
        if self.target_colour and self.target_colour in ["yellow", "red", "green", "blue"]:
            try:
                cv_img = self.cvbridge_interface.imgmsg_to_cv2(
                    img_data, desired_encoding="bgr8"
                )
                rospy.loginfo("Processing image data...")
            except CvBridgeError as e:
                print(e)
            
            height, width, _ = cv_img.shape
            crop_height = height - 400
            crop_width = 600
            crop_y = int((width/2) - (crop_width/2))
            crop_x = int((height/2) - (crop_height/2))

            crop_img = cv_img[crop_x:crop_x+crop_width, crop_y:crop_y+crop_height]
            hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_img, self.target_hsv_lower, self.target_hsv_upper)

            if np.any(mask > 0):
                res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

            m = cv2.moments(mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)

            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                self.centroid_publisher.publish(self.cy) # Publish the centroid of the detected beacon

                if abs(self.cy - crop_width / 2) < 50 and not self.beacon_detected:
                    # Check if target_colour is specified before saving the image
                    if self.target_colour:
                        self.save_image(img_data)
                    self.beacon_detected = True
                    rospy.loginfo(f"Beacon detected and image captured.")
                    self.stop_counter = 20

            if self.beacon_detected and self.stop_counter > 0:
                self.stop_counter -= 1
            else:
                self.beacon_detected = False

            if self.beacon_detected:
                cv2.imshow('task4_beacon.jpg', crop_img)
                cv2.waitKey(1)

        self.latest_img_data = img_data

    def get_latest_img_data(self):
        return self.latest_img_data
    
    def is_beacon_detected(self):
        return self.beacon_detected

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1
                self.robot_controller.set_move_cmd(0.0, 0.0)

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 30
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
                
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop' and self.stop_counter > 0:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == '__main__':
    node = TestColourSearch()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
