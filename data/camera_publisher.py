#!/opt/conda/envs/pipeline-v2/bin/python

import sys
import os
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String


if __name__ == '__main__':
    print("camera publisher node")

    rospy.init_node("camera_publisher", anonymous=True)

    img_path = "./example.png"
    image = cv2.imread(img_path)
    br = CvBridge()
    pub = rospy.Publisher("/camera/image_color", Image, queue_size=50)
    loop_rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        
        if image is not None:
            rospy.loginfo("publishing image")
            pub.publish(br.cv2_to_imgmsg(image))
        else:
            rospy.loginfo("image is none :(")

        loop_rate.sleep()
