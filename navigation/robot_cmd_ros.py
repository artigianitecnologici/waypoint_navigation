#!/usr/bin/env python

import time
import os
import socket
import math
import sys
from threading import Thread
from datetime import datetime

import rospy
import rosnode
import tf
import actionlib

try:
  import cv2
except:
  print("Cannot import OpenCV")

try:
  import numpy
except:
  print("Cannot import numpy")

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Range, Image, Joy
from control_msgs.msg import JointJog
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge, CvBridgeError

# Android phone sensors read with 'ROS Sensors Driver' App
from sensor_msgs.msg import Imu, NavSatFix, Illuminance, MagneticField

try:
    from rococo_navigation.msg import FollowPersonAction, FollowPersonGoal
    rococo_navigation_Found = True
except:
    print("rococo_navigation not found")
    rococo_navigation_Found = False

try:
    from apriltags_ros.msg import AprilTagDetectionArray
    AprilTagFound = True
except:
    print("apriltag_ros not found")
    AprilTagFound = False


AUDIO_SERVER_IP = '127.0.0.1'
AUDIO_SERVER_PORT = 9001
assock = None

use_robot = True
use_audio = True

robot_initialized = False
stop_request = False

# dir for writing log files (programs, images,... )
logdir = os.getenv('HOME')+'/playground/log/'
if not os.path.isdir(logdir):
  logdir = os.getenv('HOME')+'log/'
if not os.path.isdir(logdir):
  logdir = '/tmp/'


# Topic names

TOPIC_tag_detections = 'tag_detections'
TOPIC_scan = 'scan'
TOPIC_amcl_pose = 'amcl_pose'
TOPIC_cmd_vel = 'cmd_vel'
TOPIC_desired_cmd_vel = 'desired_cmd_vel'
TOPIC_odom = 'odom'
TOPIC_joints = 'cmd_joints_jog'
TOPIC_joy = 'joy'

ACTION_move_base = 'move_base'
TOPIC_sonar_0 = 'sonar_0' 
TOPIC_sonar_1 = 'sonar_1'
TOPIC_sonar_2 = 'sonar_2'
TOPIC_sonar_3 = 'sonar_3'
TOPIC_GROUND_TRUTH = 'base_pose_ground_truth'
TOPIC_SETPOSE = 'setpose'
TOPIC_STAGESAY = 'stage_say'

# Android sensors
TOPIC_IMU = '/android/imu'
IMU_      = None
IMU_sub   = None
TOPIC_FIX = '/android/fix'
FIX_      = None
FIX_sub   = None
TOPIC_MAG = '/android/magnetic_field'
MAG_      = None
MAG_sub   = None
TOPIC_ILL = '/android/illuminance'
ILL_      = None
ILL_sub   = None

def IMU_cb(data):
	global IMU_
	IMU_ = data
def FIX_cb(data):
	global FIX_
	FIX_ = data
def MAG_cb(data):
	global MAG_
	MAG_ = data
def ILL_cb(data):
	global ILL_
	ILL_ = data


# functions available for the programmer
def accel_gyro():
	global IMU_
	return IMU_
def sat_nav():
	global FIX_
	return FIX_
def magnetometer():
	global MAG_
	return MAG_
def illuminance():
	global ILL_
	return ILL_

# gbn navigation present
use_desired_cmd_vel=False

# Good values
tv_good = 0.2
rv_good = 0.8
tv_min = 0.1
rv_min = 0.1

move_step = 1.0

# robot pose from odometry
odom_robot_pose = None
# robot pose from localization
map_robot_pose = None
# robot pose from ground truth simulation
gt_robot_pose = None
# move_base target pose
target_pose = None
# robot velocity vector [linear.x, angular.z]
odom_robot_vel = None

move_base_running = False
ac_movebase = None 

def setMoveStep(x):
    global move_step
    move_step=x


def setMaxSpeed(x,r):
    global tv_good, rv_good
    tv_good=x
    rv_good=r


def setRobotNamePrefix(prefix):
    global TOPIC_tag_detections,TOPIC_scan,TOPIC_amcl_pose,TOPIC_cmd_vel,TOPIC_desired_cmd_vel, \
           TOPIC_odom,TOPIC_joy,TOPIC_joints,ACTION_move_base, \
           TOPIC_sonar_0,TOPIC_sonar_1,TOPIC_sonar_2,TOPIC_sonar_3, \
           TOPIC_GROUND_TRUTH, TOPIC_SETPOSE, TOPIC_STAGESAY

    TOPIC_tag_detections = prefix+'/' + TOPIC_tag_detections
    TOPIC_scan = prefix+'/'+TOPIC_scan
    TOPIC_amcl_pose = prefix+'/'+TOPIC_amcl_pose
    TOPIC_cmd_vel = prefix+'/'+TOPIC_cmd_vel
    TOPIC_desired_cmd_vel = prefix+'/'+TOPIC_desired_cmd_vel
    TOPIC_odom = prefix+'/'+TOPIC_odom
    TOPIC_joints = prefix + '/' + TOPIC_joints
    TOPIC_joy = prefix + '/' + TOPIC_joy
    ACTION_move_base = prefix+'/'+ACTION_move_base
    TOPIC_sonar_0 = prefix+'/'+TOPIC_sonar_0
    TOPIC_sonar_1 = prefix+'/'+TOPIC_sonar_1
    TOPIC_sonar_2 = prefix+'/'+TOPIC_sonar_2
    TOPIC_sonar_3 = prefix+'/'+TOPIC_sonar_3
    TOPIC_GROUND_TRUTH = prefix+'/'+TOPIC_GROUND_TRUTH
    TOPIC_SETPOSE = prefix+'/'+TOPIC_SETPOSE 
    TOPIC_STAGESAY = prefix+'/'+TOPIC_STAGESAY


def setAudioConnection(ip, port=9001):
    global AUDIO_SERVER_IP, AUDIO_SERVER_PORT
    AUDIO_SERVER_IP = ip
    AUDIO_SERVER_PORT = port

PARAM_gbnEnabled = '/gradientBasedNavigation/gbnEnabled'

def enableObstacleAvoidance(value=True):
    global use_desired_cmd_vel
    rospy.set_param(PARAM_gbnEnabled, value)
    use_desired_cmd_vel = value

def robot_stop_request(): # stop until next begin()
    global stop_request
    stop_request = True
    if (use_robot):
        stop()
    print("stop request")


# Condition Variables and Functions

tag_trigger_ = False
tag_id_ = -1
tag_distance_ = 0
tag_angle_ = 0
tag_count = 25

def tagTrigger():
    global tag_trigger_
    return tag_trigger_

def tagID():
    global tag_id_
    return tag_id_

def tagDistance():
    global tag_distance_
    return tag_distance_

def tagAngle():
    global tag_angle_
    return tag_angle_


def tag_trigger():
    return tagTrigger()

def tag_id():
    return tagID()

def tag_distance():
    return tagDistance()

def tag_angle():
    return tagAngle()


laser_center_dist = 10
laser_left_dist = 10
laser_right_dist = 10
laser_back_dist = 10

def laser_center_distance():
    global laser_center_dist
    return laser_center_dist

def getRobotPose(frame=None):
    return get_robot_pose(frame)

def get_robot_pose(frame=None): # returns [x,y,theta]
                         # frame: 'odom', 'map', 'gt'
    global odom_robot_pose, map_robot_pose, gt_robot_pose

    if frame==None: # auto detect
        if map_robot_pose is not None:
            return list(map_robot_pose)
        else:
            return list(odom_robot_pose)
    elif frame=='odom':
        return list(odom_robot_pose)
    elif frame=='map':
        return list(map_robot_pose)
    else: # frame=='gt':
        return list(gt_robot_pose)


def getRobotVel():
    return get_robot_vel()

def get_robot_vel():
    global odom_robot_vel
    return list(odom_robot_vel)

def getSpeed():
    return get_robot_vel()

def obstacleDistance(dir_deg=0):
    return obstacle_distance(dir_deg)

def obstacle_distance(dir_deg=0):
    global laser_center_dist, laser_left_dist, laser_right_dist, laser_back_dist
    if (dir_deg==0): #front
        return laser_center_dist
    elif (dir_deg==90): #left
        return laser_left_dist
    elif (dir_deg==-90 or dir_deg==270): # right
        return laser_right_dist
    elif (abs(dir_deg)==180): # back
        return laser_back_dist


def distance(p1,p2):
    dx = p1[0]-p2[0]
    dy = p1[1]-p2[1]
    dx2 = dx*dx
    dy2 = dy*dy
    return math.sqrt(dx2+dy2)


# ROS param access

def set_global_param(var, value):
    param = '/MARRtino/params/'+var
    now = rospy.Time.now()
    pd = {} 
    pd['value'] = value
    pd['timestamp'] = now.secs
    rospy.set_param(param, pd)


def get_global_param(var):
    param = '/MARRtino/params/'+var
    value = ''
    if rospy.has_param(param):
        value = rospy.get_param(param,'')
    return value

def del_global_param(var):
    param = '/MARRtino/params/'+var
    if rospy.has_param(param):
        rospy.delete_param(param)


def event():
    pd = get_global_param('event')
    if (pd==''):
        return ''
    now = rospy.Time.now()
    if (now.secs - pd['timestamp'] < 5): # last 5 seconds
        del_global_param('event')
        return pd['value']
    else:
        return ''

# ROS publishers/subscribers
cmd_pub = None # cmd_vel publisher
des_cmd_pub = None # desired_cmd_vel publisher
tag_sub = None # tag_detection subscriber
laser_sub = None # laser subscriber
odom_sub = None  # odom subscriber
joints_pub = None # joint publisher
joy_sub = None # Joystick subscriber
localizer_sub = None
sonar_sub_0 = None
sonar_sub_1 = None
sonar_sub_2 = None
sonar_sub_3 = None
stage_setpose_pub = None # Stage setpose (needs stagerosPeople)
stage_say_pub = None # Stage say (needs stagerosPeople)


# ROS Callback functions


def tag_cb(data):
    global tag_trigger_, tag_count, tag_id_, tag_distance_, tag_angle_
    v = data.detections
    if (len(v)>0):
        tag_id_ = v[0].id
        tag_distance_ = v[0].pose.pose.position.z
        tag_angle_ = math.atan2(-v[0].pose.pose.position.x,v[0].pose.pose.position.z)*180.0/math.pi

        tag_trigger_ = True
        tag_count = 3 # about seconds
        # print 'tag ',tag_id_,' distance ',tag_distance_
        # print 'tag trigger = ',tag_trigger_
    else:
        if (tag_trigger):
            tag_count = tag_count - 1
            # print 'tag count = ',tag_count
            if (tag_count==0):
                tag_trigger_ = False
                tag_id_ = -1
                tag_distance_ = 0
                tag_angle_ = 0


def laser_cb(data):
    global laser_center_dist, laser_left_dist, laser_right_dist, laser_back_dist
    nc = len(data.ranges)/2
    nr = int((data.angle_max - math.pi/2)/data.angle_increment)
    nl = len(data.ranges) - nr
    laser_center_dist = min(data.ranges[nc-10:nc+10])
    try:
        laser_left_dist = min(data.ranges[nl-10:nl+10])
        laser_right_dist = min(data.ranges[nr-10:nr+10])
    except:
        pass
        #laser_left_dist = -1
        #laser_right_dist = -1
    #print("angle min %.3f max %.3f inc %.6f" %(data.angle_min, data.angle_max, data.angle_increment))
    #print("center %.3f left %.3f right %.3f" %(laser_center_dist, laser_left_dist, laser_right_dist))


def sonar_cb(data):
    global laser_center_dist, laser_left_dist, laser_right_dist, laser_back_dist
    r = data.range  # ??? *0.75/0.265 #scale the value of the range in meters
    if(data.header.frame_id == "/sonar_frame_0"): # front
        laser_center_dist = r
    elif(data.header.frame_id == "/sonar_frame_1"): # right
        laser_right_dist = r
    elif(data.header.frame_id == "/sonar_frame_3"): # left
        laser_left_dist = r
    elif(data.header.frame_id == "/sonar_frame_2"): # back
        laser_back_dist = r


def odom_cb(data):
    global odom_robot_pose, odom_robot_vel
    # print("!!! odom cb !!!")
    if (odom_robot_pose is None):
        odom_robot_pose = [0,0,0]
    odom_robot_pose[0] = data.pose.pose.position.x
    odom_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    odom_robot_pose[2] = euler[2] # yaw
    #odomframe = data.header.frame_id

    if (odom_robot_vel is None):
        odom_robot_vel = [0,0]
    odom_robot_vel[0] = data.twist.twist.linear.x
    odom_robot_vel[1] = data.twist.twist.angular.z


def localizer_cb(data):
    global map_robot_pose
    if (map_robot_pose is None):
        map_robot_pose = [0,0,0]
    map_robot_pose[0] = data.pose.pose.position.x
    map_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    map_robot_pose[2] = euler[2] # yaw


def groundtruth_cb(data):
    global gt_robot_pose
    if (gt_robot_pose is None):
        gt_robot_pose = [0,0,0]
    gt_robot_pose[0] = data.pose.pose.position.x
    gt_robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    gt_robot_pose[2] = euler[2] # yaw



# speed/jog from Joystick
joy_cmd_vel = [0, 0]

def joy_cb(data):
    global joy_cmd_vel 
    joy_cmd_vel = [data.axes[1], data.axes[2]]

def getJoyVel():
    return joy_cmd_vel


cvbridge = None
cvimage = None

def image_cb(data):
    global cvbridge, cvimage
    # Convert image to OpenCV format
    try:
        if cvbridge is None:
            cvbridge = CvBridge()
        cvimage = cvbridge.imgmsg_to_cv2(data, "bgr8")
        #print('image')
    except CvBridgeError as e:
        print(e)



# select topic of type sensor_msgs/Image
def autoImageTopic():
    topics = rospy.get_published_topics()
    for t in topics:
        if t[1]=='sensor_msgs/Image' and 'depth' not in t[0] and '/ir/' not in t[0] \
           and 'person' not in t[0] and 'tag' not in t[0]:
            return t[0]
    return None


# Audio client

run_audio_connect = True
audio_connected = False

def audio_connect_thread():
    global run_audio_connect, assock
    print("Audio enabled, Connecting...")
    run_audio_connect = True
    timeout = 1
    while run_audio_connect and timeout>0:
        assock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            assock.connect((AUDIO_SERVER_IP, AUDIO_SERVER_PORT))
            print("Audio connected.")
            run_audio_connect = False
            audio_connected = True
        except:
            print("Cannot connect to audio server %s:%d" %(AUDIO_SERVER_IP, AUDIO_SERVER_PORT))
            time.sleep(1)
            timeout -= 1
            assock = None
    run_audio_connect = False


def get_ROS_nodes():
    try:
        nodenames = rosnode.get_node_names()
    except Exception as e:
        #print e
        return None
    return nodenames

# Begin/end

def begin(nodename='robot_cmd', init_node=True):
    global cmd_pub, des_cmd_pub, odom_sub, gt_sub, joints_pub, joy_sub, tag_sub, laser_sub, \
           sonar_sub_0, sonar_sub_1, sonar_sub_2, sonar_sub_3, \
           stage_say_pub, stage_setpose_pub, \
           odom_robot_pose, robot_initialized, stop_request, \
           use_robot, use_audio, audio_connected

    print('begin')

    stop_request = False

    if (use_audio and not audio_connected):
        # Run audio connection thread
        t = Thread(target=audio_connect_thread, args=())
        t.start()
        time.sleep(0.5)

    # if gbn node running, enable obstacle avoidance

    nn = get_ROS_nodes()
    #print(nn)
    obstav = '/gradientBasedNavigation' in nn
    enableObstacleAvoidance(obstav)
    if obstav:
      print("gbn detected: obstacle avoidance automatically enabled")

    if (robot_initialized):
        return

    # blocking function if roscore not available !!!
    # does not throw exception
    if init_node:
        rospy.init_node(nodename,  disable_signals=True)
        rospy.sleep(1)
        print("ROS node %s initialized." %nodename)

    if AprilTagFound:
        tag_sub = rospy.Subscriber(TOPIC_tag_detections, AprilTagDetectionArray, tag_cb)
    laser_sub = rospy.Subscriber(TOPIC_scan, LaserScan, laser_cb)
    sonar_sub_0 = rospy.Subscriber(TOPIC_sonar_0, Range, sonar_cb)
    sonar_sub_1 = rospy.Subscriber(TOPIC_sonar_1, Range, sonar_cb)
    sonar_sub_2 = rospy.Subscriber(TOPIC_sonar_2, Range, sonar_cb)
    sonar_sub_3 = rospy.Subscriber(TOPIC_sonar_3, Range, sonar_cb)
    localizer_sub = rospy.Subscriber(TOPIC_amcl_pose, PoseWithCovarianceStamped, localizer_cb)
    joy_sub = rospy.Subscriber(TOPIC_joy, Joy, joy_cb)
    IMU_sub = rospy.Subscriber(TOPIC_IMU, Imu,           IMU_cb)
    FIX_sub = rospy.Subscriber(TOPIC_FIX, NavSatFix,     FIX_cb)
    MAG_sub = rospy.Subscriber(TOPIC_MAG, MagneticField, MAG_cb)
    ILL_sub = rospy.Subscriber(TOPIC_ILL, Illuminance,   ILL_cb)

    if (use_robot):
        print("Robot enabled")
        cmd_pub = rospy.Publisher(TOPIC_cmd_vel, Twist, queue_size=1)
        des_cmd_pub = rospy.Publisher(TOPIC_desired_cmd_vel, Twist, queue_size=1)
        odom_sub = rospy.Subscriber(TOPIC_odom, Odometry, odom_cb)
        gt_sub = rospy.Subscriber(TOPIC_GROUND_TRUTH, Odometry, groundtruth_cb)
        joints_pub = rospy.Publisher(TOPIC_joints, JointJog, queue_size=1)
        stage_setpose_pub = rospy.Publisher(TOPIC_SETPOSE, Pose, queue_size=1, latch=True)
        stage_say_pub = rospy.Publisher(TOPIC_STAGESAY, String, queue_size=1,   latch=True)

        timeout = 3 #seconds
        print("Waiting for robot pose on topic %s... (%d seconds)" %(TOPIC_odom,timeout))
        delay = 0.25 # sec
        rate = rospy.Rate(1/delay) # Hz
        try:
            rate.sleep()
            while (odom_robot_pose is None and timeout>0):
                rate.sleep()
                timeout -= delay
        except KeyboardInterrupt:
            pass
        if (odom_robot_pose is None):
            print("Robot pose not received. Using [0,0,0]")
            odom_robot_pose = [0,0,0]  # default value
        robot_initialized = True



def end():
    global robot_initialized, stop_request

    if not robot_initialized:
        return

    if (use_robot):
        stop()
    stop_request = True

    if (use_audio):
        global run_audio_connect, audio_connected
        run_audio_connect = False
        global assock
        if assock is not None:
            assock.close()
            assock=None
            audio_connected = False

    try:
        rospy.sleep(0.5) # make sure stuff ends
    except:
        pass

    print('end')

# to unregister all the subscribers
def unregisterAll():
    #sub_XXX.unregister()
    pass


sub_image = None

def startCameraGrabber():
    global sub_image
    img_topic = autoImageTopic()
    if img_topic != None:
        print("Image topic: %s" %img_topic)
        sub_image = rospy.Subscriber(img_topic, Image, image_cb)
        time.sleep(1)



def stopCameraGrabber():
    global sub_image
    if sub_image !=  None:
        sub_image.unregister()


def getImage(tmsleep=3):
    return get_image(tmsleep)

def get_image(tmsleep=3):
    global cvimage
    startCameraGrabber() # wait 1 sec for an image
    time.sleep(tmsleep)
    stopCameraGrabber()
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%Y%m%d-%H%M%S")
    cv2.imwrite(os.getenv('MARRTINO_APPS_HOME')+'/www/viewer/img/lastimage.jpg', cvimage)
    cv2.imwrite('%s/%s.jpg' %(logdir,timestampStr), cvimage)
    #print('cvimage',cvimage)
    return cvimage

def getWebImage(objcat=None):
    return get_web_image(objcat)

def get_web_image(objcat=None):
    rchomelearnros_import()
    img = webimages.take_image(objcat)
    cv2.imwrite(os.getenv('MARRTINO_APPS_HOME')+'/www/viewer/img/lastimage.jpg', img)
    return img

# Haar detector
def findCascadeModel():
    trylist = ['/usr/share/opencv/', '/opt/ros/kinetic/share/OpenCV-3.3.1-dev/' ]
    for t in trylist:
        f = t + 'haarcascades/haarcascade_frontalface_default.xml'
        if os.path.isfile(f):
            return cv2.CascadeClassifier(f)
    return None

faceCascade = None

def faceDetection(img):
    return face_detection(img)

def face_detection(img):
    global faceCascade
    if faceCascade is None:
        faceCascade = findCascadeModel()
        if faceCascade is None:
            print("ERROR Cannot find Haar cascade model")
            return -1
    if img is None:
        print("ERROR No image")
        return -1
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect faces in the image
    faces = faceCascade.detectMultiScale(gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30)
    )
    return len(faces)



# Object recognition with mobilenet

# REQUIRES tensorflow 2.x !!!

# rc-home-learn-ros import
rchomelearnros_imported = False
mobilenet_objrec = None 
webimages = None

def rchomelearnros_import():
    global rchomelearnros_imported, mobilenet_objrec, webimages
    if rchomelearnros_imported:
        return

    path = None
    try:
        import rospkg
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        path = rospack.get_path('rc-home-edu-learn-ros')
    except Exception as e:
        #print(e)
        path = os.getenv('HOME')+'/src/rc-home-edu-learn-ros'
    print('rc-home-edu-learn-ros path: %s' %path) 

    try:
        sys.path.append(path+'/rchomeedu_vision/scripts')
        import mobilenet_objrec, webimages
        rchomelearnros_imported = True
    except Exception as e:
        print(e)
        print('Cannot import mobilenet_objrec, webimages modules')


# local instance of mobilenet_objrec

monet = None

def mobilenetObjrec(img):
    return mobilenet_objrec(img)

def mobilenet_objrec(img):
    global monet
    if monet is None:
        rchomelearnros_import()
        try:
            monet = mobilenet_objrec.MNetObjRec()
        except Exception as e:
            print(e)
            return 'ERROR Mobilenet not available'
    
    r = monet.evalCVImage(img)
    return r


# Object recognition with mobilenet server

# DOES NOT REQUIRE tensorflow on the client side !!!

def send_image(image, width, height, server, port):
    if image is not None:
        sendimage = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        sendimage = cv2.resize(sendimage, (width,height))

        sdata = numpy.array(sendimage)
        stringData = sdata.tostring()

        try:  
            sock = socket.socket()   # connection to server
            sock.connect((server, port))
            (h,w,c) = sendimage.shape
            print("Sending %dx%d image " %(w,h))
            sock.sendall("RGB %d %d\n\r" %(w,h))
            rospy.sleep(0.2)
            sock.sendall(stringData);
            rdata = sock.recv(256)
            rdata = rdata.strip().decode('UTF-8')
            print("Received: %s" %rdata)
            sock.close()
            vdata = rdata.split(" ")
            rval = (vdata[0], float(vdata[1]))
        except Exception as e:
            print(e)
            print("Cannot send image to %s:%d" %(server, port))
            rval = ("send-error", 0.0)

        return rval


def mobilenetObjrecClient(img, server='localhost', port=9300):
    # send image to sever
    print("Sending image to server %s:%s" %(server,port))
    w = 224
    h = 224
    (label,conf) = send_image(img, w, h, server, port)

    print("result: %s %.2f" %(label,conf))

    return (label,conf)



def ready():
    global robot_initialized
    return robot_initialized


# check if program can run now
def marrtino_ok():
    return marrtinoOK()

def marrtinoOK():
    global robot_initialized, stop_request
    return robot_initialized and not stop_request and not rospy.is_shutdown()


# Robot motion

def setSpeed(lx,az,tm,stopend=False):
    return set_speed(lx,az,tm,stopend)

def set_speed(lx,az,tm,stopend=False):
    global cmd_pub, des_cmd_pub, use_desired_cmd_vel, stop_request, tv_good, rv_good

    if (stop_request and (lx!=0.0 or az!=0.0)):
        raise Exception("setSpeed called in stop_request mode")

    delay = 0.05 # sec
    rate = rospy.Rate(1/delay) # Hz
    cnt = 0.0
    msg = Twist()
    msg.linear.x = lx
    msg.angular.z = az
    msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y =  0
    while not rospy.is_shutdown() and cnt<tm and not stop_request:
        if (use_desired_cmd_vel):
            des_cmd_pub.publish(msg)
        else:
            cmd_pub.publish(msg)
        cnt = cnt + delay
        try:
            rate.sleep()
        except KeyboardInterrupt:
            print("User KeyboardInterrupt")
            return False
    if (stopend):
        msg.linear.x = 0
        msg.angular.z = 0
        cmd_pub.publish(msg)
        try:
            rate.sleep()
        except:
            pass
    return True

def setSpeed4W(fl,fr,bl,br,tm,stopend=False):

    vlimit = 0.3 # limit 0.3 m/s
    if math.fabs(fl)>vlimit:
        fl = fl / math.fabs(fl) * vlimit
    if math.fabs(fr)>vlimit:
        fr = fr / math.fabs(fr) * vlimit
    if math.fabs(bl)>vlimit:
        bl = bl / math.fabs(bl) * vlimit
    if math.fabs(br)>vlimit:
        br = br / math.fabs(br) * vlimit

    fln = - fl * 0.02
    frn = + fr * 0.02
    bln = + bl * 0.02
    brn = - br * 0.02

    cnt = 0.0
    delay = 0.1 # sec
    rate = rospy.Rate(1/delay) # Hz

    msg = JointJog()
    msg.joint_names = ["front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"]
    msg.velocities = [fln,frn,bln,brn]
    msg.duration = delay

    while not rospy.is_shutdown() and cnt<=tm and not stop_request:
        joints_pub.publish(msg)
        cnt = cnt + delay
        try:
            rate.sleep()
        except KeyboardInterrupt:
            return False

    if (stopend):
        msg.velocities = [0,0,0,0]
        joints_pub.publish(msg)
        try:
            rate.sleep()
        except:
            pass
    return True


def stop():
    global cmd_pub, joints_pub, move_base_running
    print('stop')
    if (move_base_running):
        exec_movebase_stop()
    setSpeed(0,0,0.2,True);
    setSpeed4W(0,0,0,0,0.2,True);

def forward(r=1):
    global tv_good
    print('forward %.2f' %r)
    v = exec_move_REL(move_step*r)
    return v
    
def backward(r=1):
    print('backward %.2f' %r)
    return exec_move_REL(-move_step*r)


def left(r=1):
    print('left %.2f' %r)
    return exec_turn_REL(90*r)


def right(r=1):
    print('right %.2f' %r)
    return exec_turn_REL(-90*r)


# set stage pose
def stage_setpose(gx,gy,gth_deg):
    global stage_setpose_pub

    p = Pose()
    p.position.x=gx
    p.position.y=gy
    p.position.z=0
    th = gth_deg * math.pi / 180.0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, th)
    #type(pose) = geometry_msgs.msg.Pose
    p.orientation.x = quaternion[0]
    p.orientation.y = quaternion[1]
    p.orientation.z = quaternion[2]
    p.orientation.w = quaternion[3]

    stage_setpose_pub.publish(p)


# map frame goto (requires localization)
def goto(gx, gy, gth_deg):
    return exec_movebase(gx, gy, gth_deg)

# map frame goto (requires localization)
def gotoPose(target_pose):
    return exec_movebase(target_pose[0], target_pose[1], target_pose[2])

# map frame goto (requires localization)
def gotoLabel(target_label):
    # TODO
    pname = "/map_server/%s/gotopose" %target_label
    if rospy.has_param(pname):
      p = rospy.get_param(pname)
      return exec_movebase(p[0], p[1], p[2])
    else:
      rospy.logerr("Label %s not found." %target_label)
      return False


# odom frame direct control (no path planning)
def gotoTarget(gx, gy, frame='odom'):
    goto_target(gx, gy, frame)


# odom frame direct control (no path planning)
def goto_target(gx, gy, frame='odom'):
    exec_goto_target(gx, gy, frame)

# person follow


def start_follow_person(max_vel = 0.25): # non-blocking
    exec_follow_person_start(max_vel)

def stop_follow_person():
    exec_follow_person_stop()

# Turn

def turn(deg, ref='REL', frame='odom'):
    if ref=='REL':
        deg = NORM_180(deg)
    print('turn %s %.2f frame %s' %(ref,deg,frame))
    if ref=='REL':
        return exec_turn_REL(deg,frame)
    else:
        return exec_turn_ABS(deg,frame)

def dsleep(d):
    try:
        rospy.sleep(d)
    except KeyboardInterrupt:
        return False
    return True

# Wait

def wait(r=1):
    global stop_request
    #print('wait %.1f' %r)

    if (r<=0):
        return dsleep(0.1)
    elif (r<1):
        return dsleep(r)
    else:
        t = 0
        e = True
        while t<r and not stop_request and e:
            d = min(1,r-t)
            e = dsleep(d)
            t += d
            print("wait ... %f < %f  %r  %r " %(t,r, stop_request, e))
        return e


# Sounds

def sound(name):
    global assock
    print('sound %s' %name)
    try:
        assock.send('SOUND %s\n\r' %name)
        time.sleep(0.5)
        data = assock.recv(80)
        print(data)
    except:
        pass

def bip(r=1):
    for i in range(0,r):
        sound('bip')

def bop(r=1):
    for i in range(0,r):
        sound('bop')

def boom(r=1):
    for i in range(0,r):
        sound('boom')

# TTS

def say(text, language='en'):
    global assock
    print('say %s [%s]' %(text,language))
    lstr = 'en-US'
    if (language!='en'):
        lstr = language+'-'+language.upper()
    stage_say(text)
    try:
        assock.send('TTS[%s] %s\n\r' %(lstr,text))
        rospy.sleep(1)
        data = assock.recv(80)
        print(data)
    except:
        rospy.sleep(3)
        pass
    stage_say("")
    

def stage_say(text, language='en'):
    global stage_say_pub
    s = String()
    s.data = text
    stage_say_pub.publish(s)


# ASR

def asr():
    global assock, stop_request
    #print 'ASR received: ',
    try:
        data = ''
        while data=='' and not stop_request:
            assock.send('ASR\n\r')  # ask for ASR results
            time.sleep(0.5)
            data = assock.recv(160)
            data = data.strip()
        #print data
        return data
    except:
        return ''


# MODIM

mws = None  # MODIM websocket connection

try:
    sys.path.append(os.getenv('MODIM_HOME')+"/src/GUI")
    from ws_client import *
    mws = ModimWSClient()
except:
    print("No MODIM found!")

# example: show_image('red.jpg', 'default')


def showImage(value, which='default'):
    show_image(value, which)

def show_image(value, which='default'):
    global mws
    if mws!=None:
        cstr = 'im.executeModality("image_%s", "img/%s")' %(which,value)
        #print(cstr)
        r = mws.csend(cstr)
        print(r)


def showText(value, which='default'):
    show_text(value, which)

def show_text(value, which='default'):
    global mws
    if mws!=None:
        cstr = 'im.executeModality("text_%s", "%s")' %(which,value)
        #print(cstr)
        r = mws.csend(cstr)
        print(r)


# Precise move and turn

# Angle functions

def DEG2RAD(a):
    return a*math.pi/180.0

def RAD2DEG(a):
    return a/math.pi*180.0

def NORM_180(a):
    if (a>180):
        return a-360
    elif (a<-180):
        return a+360
    else:
        return a


def NORM_PI(a):
    if (a>math.pi):
        return a-2*math.pi
    elif (a<-math.pi):
        return a+2*math.pi
    else:
        return a

def norm_target_angle(a):
    if (abs(NORM_PI(a-0))<0.3):
        return 0;
    elif (abs(NORM_PI(a-math.pi/2.0))<0.3):
        return math.pi/2;
    elif (abs(NORM_PI(a-math.pi))<0.3):
        return math.pi;
    elif (abs(NORM_PI(a-3*math.pi/2.0))<0.3):
        return -math.pi/2;
    else:
        return a;



def exec_turn_ABS(th_deg, frame='odom'):

    robot_pose = get_robot_pose(frame)
    current_th_deg = RAD2DEG(robot_pose[2])   # deg
    a_deg = NORM_180(th_deg - current_th_deg)
    #print("Turn rel %.1f" %a_deg)
    return exec_turn_REL(a_deg, 'odom')


def exec_turn_REL(th_deg, frame='odom'):
    global rv_good, rv_min

    robot_pose = get_robot_pose(frame)
    current_th = robot_pose[2]
    #print("TURN -- currentTh: %.1f -- targetTh %.1f" %(RAD2DEG(current_th), RAD2DEG(current_th) + th_deg))
    #print("TURN -- to-normalize RAD: %.1f" %(current_th + DEG2RAD(th_deg)))
    target_th = norm_target_angle(current_th + DEG2RAD(th_deg))
    #print("TURN -- currentTh: %.1f -- targetTh %.1f" %(RAD2DEG(current_th), RAD2DEG(target_th)))

    r = True

    rv_nom = rv_good 
    if (th_deg < 0):
        rv_nom *= -1

    dth = abs(NORM_PI(target_th-current_th))

    #print("TURN -- dTh %.2f norm_PI: %.2f" %(current_th-target_th,dth))

    last_dth = dth
    #print("TURN -- last_dth %.2f" %(last_dth))
    while (dth>rv_min/8.0 and last_dth>=dth):
        rv = rv_nom
        if (dth<0.8):
            rv = rv_nom*dth/0.8
        if (abs(rv)<rv_min):
            rv = rv_min*rv/abs(rv)
        tv = 0.0
        if setSpeed(tv, rv, 0.1, False):
            robot_pose = get_robot_pose(frame)
            current_th = robot_pose[2]
            dth = abs(NORM_PI(target_th-current_th))
            if (dth < last_dth or dth>0.3): # to avoid oscillation close to 0
                last_dth = dth
        else:
            print("turn action canceled by user")
            r = False
            dth=0
        #print("TURN -- POS: %.1f %.1f %.1f -- targetTh %.1f DTH %.2f -- VEL: %.2f %.2f" %(robot_pose[0], robot_pose[1], RAD2DEG(current_th), RAD2DEG(target_th), RAD2DEG(dth), tv, rv))
    #print("TURN -- dth %.2f - last_dth %.2f" %(dth,last_dth))
    setSpeed(0.0,0.0,0.1)
    #print 'TURN -- end'
    return r


def exec_move_REL(tx, frame='odom'):
    global tv_good

    robot_pose = get_robot_pose(frame)
    start_pose = list(robot_pose)
    tv_nom = tv_good 
    r = True
    if (tx < 0):
        tv_nom *= -1
        tx *= -1
    dx = tx - distance(start_pose,robot_pose)  
    while (dx>0.01):
        tv = tv_nom
        if (dx<0.5):
            tv = tv_nom*dx/0.5
        if (abs(tv)<tv_min):
            tv = tv_min*tv/abs(tv)
        rv = 0.0
        if setSpeed(tv, rv, 0.1, False):
            robot_pose = get_robot_pose(frame)
            dx = tx - distance(start_pose, robot_pose)
        else:
            print("move action canceled by user")
            r = False
            dx = 0
        #print("MOVE -- POS: %.1f %.1f %.1f -- targetTX %.1f DX %.1f -- VEL: %.2f %.2f" %(robot_pose[0], robot_pose[1], RAD2DEG(robot_pose[2]), tx, dx, tv, rv))
    setSpeed(0.0,0.0,0.1)
    return r


def exec_goto_target(gx,gy, frame='odom'):
    global tv_good, rv_good, tv_min, rv_min, odom_robot_pose, map_robot_pose

    robot_pose = get_robot_pose(frame)
    goal_pose = [gx,gy,0]
    r = True
    dx = distance(goal_pose,robot_pose)
    while (dx>0.2):
        tv = tv_good
        if (dx<0.5):
            tv = tv*dx/0.5
        if (abs(tv)<tv_min):
            tv = tv_min*tv/abs(tv)

        current_th = robot_pose[2]
        th_goal = math.atan2(gy-robot_pose[1],gx-robot_pose[0])

        #print("GOTO -- th_target: %.2f" %(RAD2DEG(th_goal)))

        dth = NORM_PI(th_goal-current_th)
        if abs(dth)>0.1:
            rv = rv_good * dth/abs(dth)
        else:
            rv = 0
        if (abs(dth)>0.8):
            tv = tv_min
        if (abs(dth)<0.8):
            rv = rv*abs(dth)/0.8
        if (abs(rv)<rv_min and abs(rv)>0):
            rv = rv_min*rv/abs(rv)

        if setSpeed(tv, rv, 0.1, False):
            robot_pose = get_robot_pose(frame)
            dx = distance(goal_pose, robot_pose)
        else:
            r = False
            print("goto_target action canceled by user")
            dx = 0
        #print("GOTO -- POS: %.1f %.1f %.1f -- target %.1f %.1f -- dx: %.1f dth: %.1f -- VEL: %.2f %.2f" %(robot_pose[0], robot_pose[1], RAD2DEG(robot_pose[2]), gx, gy, dx, dth, tv, rv))
    setSpeed(0.0,0.0,0.1)

    return r

def dist_from_goal():
    global target_pose
    if target_pose != None:
        p = get_robot_pose()
        return math.sqrt(math.pow(p[0]-target_pose[0],2)+math.pow(p[1]-target_pose[1],2))
    else:
        return -1


def start_movebase_pose(target_pose): # non-blocking
    start_movebase(target_pose[0], target_pose[1], target_pose[2])


def start_movebase(gx, gy, gth_deg): # non-blocking
    global ac_movebase, move_base_running, target_pose

    if (ac_movebase == None):
        ac_movebase = actionlib.SimpleActionClient(ACTION_move_base,MoveBaseAction)
        timeout = rospy.Duration(5)
        if not ac_movebase.wait_for_server(timeout):
            print("ERROR start_movebase: Cannot connect with move_base server")
            return False

    target_pose = [gx, gy, gth_deg/180.0*math.pi]

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = gx
    goal.target_pose.pose.position.y = gy
    yaw = gth_deg/180.0*math.pi
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    
    ac_movebase.send_goal(goal)
    move_base_running = True
    print("move_base action started: target %r" %(target_pose))
    rospy.sleep(0.2)
    return True

def movebase_running():
    global ac_movebase, move_base_running
    r = False
    if move_base_running:
        try:
            r = not ac_movebase.wait_for_result(rospy.Duration(1))
        except KeyboardInterrupt:
            print("movebase action canceled by user")
            r = False
    return r


def movebase_step(delay):  # executes one move_base step of delay seconds
                           # return [finish, success] 
                           # finish = True if action is terminated
                           # success = True if goal has been reached
    global ac_movebase, move_base_running, target_pose

    finish = False
    success = False

    rospy.sleep(delay)

    try:
        st = ac_movebase.get_state()
        #res = ac_movebase.wait_for_result(rospy.Duration(delay))  # true: action finished

        #res2 = ac_movebase.get_result()

        #print("-- movebase wait_for_result: %r" %res)
        #print("-- movebase result: %s" %res2)
        #print("-- movebase state: %s" %st)
        # state  1: running, 3: succeed, 4: abort
    

        if rospy.has_param('/move_base_node/TrajectoryPlannerROS/xy_goal_tolerance'):
            gt = rospy.get_param('/move_base_node/TrajectoryPlannerROS/xy_goal_tolerance')
        elif rospy.has_param('/move_base_node/DWAPlannerROS/xy_goal_tolerance'):
            gt = rospy.get_param('/move_base_node/DWAPlannerROS/xy_goal_tolerance')
        else:
            gt = 0.25

        #gt *= 1.5 # margin needed to avoid deadlocks

        #d = dist_from_goal()
        #print("-- d: %f < %f" %(d,gt))

        if st==1 and target_pose[2]>=999 and dist_from_goal()<gt:
            print('Goal reached, ignoring orientation')
            finish = True
            success = True
        elif st==3:
            finish = True
            success = True
        elif st==4:
            finish = True
            success = False
    except KeyboardInterrupt:
        print("move_base action canceled by user")
        finish = True
        success = False

    if finish:
        print("move_base action finished. success: %r" %success)

    return (finish, success)


def exec_movebase(gx, gy, gth_deg):  # blocking
    global ac_movebase, move_base_running, target_pose

    r = start_movebase(gx, gy, gth_deg)
    if not r:
        return False

    success = True

    delay = 0.5
    while move_base_running:
        finish, success = movebase_step(delay)
        if finish:  # action is terminated
            exec_movebase_stop()
    
    print('Move action completed. Success: %r' %success)
    move_base_running = False
    target_pose = None
    return success


def movebase_stop():
    exec_movebase_stop()


def exec_movebase_stop():
    global ac_movebase, move_base_running
    move_base_running = False
    target_pose = None
    if (ac_movebase == None):
        ac_movebase = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        timeout = rospy.Duration(5)
        if not ac_movebase.wait_for_server(timeout):
            print("ERROR stop_movebase: Cannot connect with move_base server")
            return
    ac_movebase.cancel_all_goals()


ac_follow_person = None  # action client
follow_person_running = False  # running flag
PERSON_FOLLOW_ACTION = 'follow_person'

def exec_follow_person_start(max_vel):
    global ac_follow_person, follow_person_running

    if not rococo_navigation_Found:
        print("Action %s not available"  %PERSON_FOLLOW_ACTION)
        return

    if (ac_follow_person == None):
        ac_follow_person = actionlib.SimpleActionClient(PERSON_FOLLOW_ACTION,FollowPersonAction)

    print('Waiting for action server %s ...' %PERSON_FOLLOW_ACTION)
    ac_follow_person.wait_for_server()
    print('Done')

    goal = FollowPersonGoal()
    goal.person_id = 0;      # unused so far
    goal.max_vel = max_vel;  # m/s
    ac_follow_person.send_goal(goal)

    print("Follow person START")
    follow_person_running = True


def exec_follow_person_stop():
    global ac_follow_person, follow_person_running

    if not rococo_navigation_Found:
        print("Action %s not available"  %PERSON_FOLLOW_ACTION)
        return

    if (ac_follow_person == None):
        ac_follow_person = actionlib.SimpleActionClient(PERSON_FOLLOW_ACTION,FollowPersonAction)
    print('Waiting for action server %s ...' %PERSON_FOLLOW_ACTION)
    ac_follow_person.wait_for_server()
    print('Done')
    ac_follow_person.cancel_all_goals()

    print("Follow person STOP")

    follow_person_running = False



