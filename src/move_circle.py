#!/usr/bin/env python3

import rospy 


class Circle():

    def __init__(self):
        self.node_name = "self_move"

        self.pub = rospy.Publisher() 
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = 5

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        

        self.ctrl_c = True

    def main(self):
        while not self.ctrl_c:
            
            


if __name__ == '__main__':
    node.Circle()
    node.main()