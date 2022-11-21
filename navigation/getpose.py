import math, time, sys

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

import tf

TOPIC_odom = '/odom'
TOPIC_amcl_pose = '/amcl_pose'
TOPIC_ground_truth = '/base_pose_ground_truth'

odom_pose = None
map_pose = None
ground_truth_pose = None
odomframe = ''
odomcount = 0 
mapframe = ''
mapcount = 0 


def odom_cb(data):
    global odom_pose, odomcount, odomframe
    if (odom_pose is None):
        odom_pose = [0,0,0]
    odom_pose[0] = data.pose.pose.position.x
    odom_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    odom_pose[2] = euler[2] # yaw
    odomcount += 1
    odomframe = data.header.frame_id

def localizer_cb(data):
    global map_pose, mapcount, mapframe
    if (map_pose is None):
        map_pose = [0,0,0]
    map_pose[0] = data.pose.pose.position.x
    map_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    map_pose[2] = euler[2] # yaw
    mapcount += 1
    mapframe = data.header.frame_id

def groundtruth_cb(data):
    global ground_truth_pose
    if (ground_truth_pose is None):
        ground_truth_pose = [0,0,0]
    ground_truth_pose[0] = data.pose.pose.position.x
    ground_truth_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    ground_truth_pose[2] = euler[2] # yaw



def DEG(a):
    return a*180.0/math.pi

def pose_str(p):
    return "%.2f %.2f %.2f DEG" %(p[0], p[1], DEG(p[2]))


# main
if __name__ == "__main__":

    rospy.init_node('getpose', disable_signals=True)
    rospy.sleep(1)

    if len(sys.argv)>1:
        print("Get pose for %s" %sys.argv[1])
        TOPIC_odom = "/" + sys.argv[1] + TOPIC_odom
        TOPIC_amcl_pose = "/" + sys.argv[1] + TOPIC_amcl_pose
        TOPIC_ground_truth = "/" + sys.argv[1] + TOPIC_ground_truth

    odom_sub = rospy.Subscriber(TOPIC_odom, Odometry, odom_cb)
    localizer_sub = rospy.Subscriber(TOPIC_amcl_pose, PoseWithCovarianceStamped, localizer_cb)
    gt_sub = rospy.Subscriber(TOPIC_ground_truth, Odometry, groundtruth_cb)

    rate = rospy.Rate(10)
    timesteps = 10
    while not rospy.is_shutdown() and timesteps>0:
        rate.sleep()
        timesteps -= 1

    if odom_pose is not None:
        print("Odom  Robot  pose: %s"  %(pose_str(odom_pose)))

    if map_pose is not None:
        print("Map   Robot  pose: %s"  %(pose_str(map_pose)))

    if ground_truth_pose is not None:
        print("Ground truth pose: %s"  %(pose_str(ground_truth_pose)))



    odom_sub.unregister()
    localizer_sub.unregister()
    gt_sub.unregister()



