#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import json

pub = rospy.Publisher('arm_difference', String, queue_size=20)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(1) # 10hz
while not rospy.is_shutdown():
  difference = {
    "orientation":{
      "x":0,
      "y":0,
      "z":0,
    },
    "position":{
      "x":-1,
      "y":0,
      "z":0
    }
  }
  pub.publish(json.dumps(difference))
  rate.sleep()