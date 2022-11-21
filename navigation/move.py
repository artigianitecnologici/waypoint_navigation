from robot_cmd_ros import *
import robot_cmd_ros
import sys
import os
import time
import math
import argparse

import rospy
import actionlib
from threading import Thread

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

sys.path.append(os.getenv("MARRTINO_APPS_HOME")+"/program")


robot_cmd_ros.use_audio = False


ACTION_move_base = '/move_base'
TOPIC_odom = '/odom'
TOPIC_cmd_vel = '/cmd_vel'
FRAME_map = 'map'
FRAME_base = 'base_frame'


def cmdvel_cb(data):
    global robot_vel
    robot_vel[0] = data.linear.x
    robot_vel[1] = data.angular.z


def turntogoal(target_pose):
    r = True

    p = getRobotPose()

    if math.fabs(target_pose[1]-p[1]) + math.fabs(target_pose[0]-p[0]) < 0.5:
        return True

    ad = math.atan2(target_pose[1]-p[1], target_pose[0]-p[0])
    th_deg = (ad-p[2])*180/math.pi
    if math.fabs(th_deg) > 30:
        r = turn(th_deg)

    return r

# blocking function (no monitoring)


def do_move_no_monitor(target_pose):
    r = turntogoal(target_pose)
    if r:
        r = goto(target_pose)
    return r

# blocking function with monitoring


def do_move(target_pose):

    r = turntogoal(target_pose)
    if not r:
        return False

    start_movebase_pose(target_pose)

    delay = 0.25  # sec
    rate = rospy.Rate(1/delay)  # Hz
    rate.sleep()

    # wait for reaching the goal
    while movebase_running() and not rospy.is_shutdown():
        #finish, successs = movebase_step(delay)
        #print("Distance to goal: %.2f" %dist_from_goal())
        pass

    movebase_stop()

    if dist_from_goal() < 0.5:
        print('Goal reached')
        r = True
    else:
        print('Goal not reached')
        r = False

    return r


# main
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='move_base navigation')
    parser.add_argument('GX', type=float, help='Goal X')
    parser.add_argument('GY', type=float, help='Goal Y')
    parser.add_argument('GTH', nargs='?',  type=float,
                        default=1001, help='Goal Theta [deg]')

    args = parser.parse_args()
    target_pose = [args.GX, args.GY, args.GTH]

    rospy.init_node('move', disable_signals=True)

    #cmd_vel_sub = rospy.Subscriber(TOPIC_cmd_vel, Twist, cmdvel_cb)

    time.sleep(0.2)

    begin(init_node=False)

    # set target orientation as atan angle between current pose and target pose
    if (args.GTH > 1000):
        p = getRobotPose()
        target_pose[2] = math.atan2(
            target_pose[1]-p[1], target_pose[0]-p[0])*180/math.pi

    do_move(target_pose)

    end()

    # cmd_vel_sub.unregister()

    print('Quit')

    sys.exit(0)
