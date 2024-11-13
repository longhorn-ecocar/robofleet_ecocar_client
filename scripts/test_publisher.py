#!/usr/bin/env python
import math
import random
import rospy
from roslib.message import get_message_class

from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

seed = random.random() * 100000

def talker():
    # avoid that whole "make a ROS package" issue
    # make sure ROS_PACKAGE_PATH contains amrl_msgs
    RobofleetStatus = get_message_class("amrl_msgs/RobofleetStatus")
    if RobofleetStatus is None:
        raise Exception("Ensure that amrl_msgs is on ROS_PACKAGE_PATH")
    Localization2DMsg = get_message_class("amrl_msgs/Localization2DMsg")
    Odometry = get_message_class("nav_msgs/Odometry")

    # ecocar data classes
    SensorStatus = get_message_class("amrl_msgs/SensorStatus")
    SensorHealth = get_message_class("amrl_msgs/SensorHealth")
    SystemHealth = get_message_class("amrl_msgs/SystemHealth")
    SystemLog = get_message_class("amrl_msgs/SystemLog")
    CACCStatus = get_message_class("amrl_msgs/CACCStatus")

    status_pub = rospy.Publisher("status", RobofleetStatus, queue_size=1)
    odom_pub = rospy.Publisher("odometry/raw", Odometry, queue_size=1)
    loc_pub = rospy.Publisher("localization", Localization2DMsg, queue_size=1)

    # ecocar publishers
    sensor_health_pub = rospy.Publisher('/leva/sensorhealth', SensorHealth, queue_size=1)
    system_health_pub = rospy.Publisher('/leva/systemhealth', SystemHealth, queue_size=1)
    system_log_pub = rospy.Publisher('/leva/systemlog', SystemLog, queue_size=1)
    cacc_status_pub = rospy.Publisher('/leva/caccstatus', CACCStatus, queue_size=1)
    image_pub = rospy.Publisher('/leva/birdseyeview/image_raw/compressed', CompressedImage, queue_size=1)
    image_filepath = 'images/BEVMockup.png'
    bridge = CvBridge()

    rospy.init_node("test_publisher", anonymous=True)
    rate = rospy.Rate(1)

    curr = 0
    while not rospy.is_shutdown():
        t = rospy.get_time() + seed

        rf_status = RobofleetStatus()
        rf_status.battery_level = math.sin(t / 5) * 0.5 + 0.5
        rf_status.is_ok = True
        rf_status.location = "cyberspace"
        rf_status.status = "Testing"

        odom = Odometry()
        odom.pose.pose.position.x = math.sin(t / 10) * 5 + 5
        odom.pose.pose.position.y = math.sin(t / 10 + 0.5) * 5 + 5
        odom.pose.pose.position.z = math.sin(t / 10 + 1) * 5 + 5
        odom.twist.twist.linear.x = 1
        odom.twist.twist.linear.y = 2
        odom.twist.twist.linear.z = 3

        loc = Localization2DMsg()
        loc.map = "UT_Campus"
        loc.pose.x = math.sin(t / 50) * 100 + 100
        loc.pose.y = math.cos(t / 50) * -100 - 100
        loc.pose.theta = t / 10

        # ecocar data
        # ouster status
        ouster_status = SensorStatus()
        ouster_status.sensorid = "ouster"
        ouster_status.frequency = 10
        ouster_status.std = 10
        ouster_status.packet_size = 10
        ouster_status.status = curr % 3 

        # left camera status
        left_camera_status = SensorStatus()
        left_camera_status.sensorid = "left_camera"
        left_camera_status.frequency = 10
        left_camera_status.std = 10
        left_camera_status.packet_size = 10
        left_camera_status.status = (curr + 1) % 3

        # right camera status
        right_camera_status = SensorStatus()
        right_camera_status.sensorid = "right_camera"
        right_camera_status.frequency = 10
        right_camera_status.std = 10
        right_camera_status.packet_size = 10
        right_camera_status.status = (curr + 2) % 3

        # live vehicle feed
        cv_image = cv2.imread(image_filepath)
        compressed_msg = bridge.cv2_to_compressed_imgmsg(cv_image)
        compressed_msg.header.stamp = rospy.Time.now()

        # sensor health
        sensor_health_status = SensorHealth()
        sensor_health_status.healths = [ouster_status, left_camera_status, right_camera_status]

        # system health
        system_health_status = SystemHealth()
        system_health_status.pcm_propulsion = (curr + 1) % 3
        system_health_status.pcm_highvoltage = curr % 3
        system_health_status.cav_longitudinal = (curr + 2) % 3
        system_health_status.cav_lateral = (curr + 1) % 3
        system_health_status.cav_v2x = curr % 3

        # system log
        system_log_status = SystemLog()
        system_log_status.log = "Hello World " + str(curr)

        curr += 1

        # CACCStatus
        cacc_status_status = CACCStatus()
        cacc_status_status.status = curr % 3

        rospy.loginfo("publishing")
        status_pub.publish(rf_status)
        odom_pub.publish(odom)
        loc_pub.publish(loc)

        # ecocar publishers
        # sensor_health_pub.publish(sensor_health_status)
        system_health_pub.publish(system_health_status)
        system_log_pub.publish(system_log_status)
        cacc_status_pub.publish(cacc_status_status)
        image_pub.publish(compressed_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
