#!/opt/conda/envs/pipeline-v2/bin/python

import sys
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

br = CvBridge()

def callback(msg):
    print("received image!")
    image = br.imgmsg_to_cv2(msg)
    

if __name__ == '__main__':
    print("camera subscriber node")

    rospy.init_node('camera_subscriber', anonymous=True)

    loop_rate = rospy.Rate(1)
    rospy.Subscriber("/camera/image_color", Image, callback)

    rospy.spin() # keep python alive
