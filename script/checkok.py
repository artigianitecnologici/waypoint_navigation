#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import sys
import csv
import tf
from sensor_msgs.msg import Range 
from std_msgs.msg import String

obstacle_distance = 999

def range_cb(msg):
    global obstacle_distance
    obstacle_distance = msg.range

def getRangeObstacle():  
    global obstacle_distance
    return obstacle_distance 


def checkok():
    pub = rospy.Publisher('ready', String, queue_size=10)
    range_sub = rospy.Subscriber('/teraranger_evo_mini/range',Range,range_cb)
    rospy.init_node('checkok', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        obstacle = getRangeObstacle()
        if (obstacle  < 0.50):
            print "ostacolo laser :",obstacle
            cmd_str = "OK"  
            pub.publish(cmd_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        checkok()
    except rospy.ROSInterruptException:
        pass
