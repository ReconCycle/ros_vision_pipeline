#!/opt/conda/envs/pipeline-v2/bin/python

import sys
import os
import rospy
from std_msgs.msg import String
import torch

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    print("blah")
    print("starting python talker")

    cuda_device = torch.cuda.current_device()
    print("current cuda device:", cuda_device)
    print("device name:", torch.cuda.get_device_name(cuda_device))
    print("device count:", torch.cuda.device_count())
    print("cuda available:", torch.cuda.is_available())


    try:
        talker()
    except rospy.ROSInterruptException:
        pass