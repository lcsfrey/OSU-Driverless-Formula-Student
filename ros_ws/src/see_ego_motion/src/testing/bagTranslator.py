#!/usr/bin/env python

import rospy
import threading

from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, TwistWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

if __name__ == "__main__":
    try:
        lock = threading.Lock()
        rospy.init_node("see_ego_motion_translator")
        vectornavGPS = rospy.Publisher("vectornav/GPS", NavSatFix, queue_size=1)
        vectornavOdom = rospy.Publisher("vectornav/Odom", Odometry, queue_size=1)

        outMsg = Odometry() #pose not used
        outMsg.header = Header()
        lock = threading.Lock()

        def sub_optical(msg):
            global outMsg
            global lock
            lock.acquire()
            angleVel = outMsg.twist.twist.angular.z
            outMsg.twist.twist.linear = msg.twist.linear
            outMsg.twist.covariance = [ #really small covariance
                0.01, 0, 0, 0, 0, 0,
                0, 0.01, 0, 0, 0, 0,
                0, 0, 0.01, 0, 0, 0, 
                0, 0, 0, 0.01, 0, 0,
                0, 0, 0, 0, 0.01, 0, 
                0, 0, 0, 0, 0, outMsg.twist.covariance[35]
            ]
            outMsg.twist.twist.angular.z = angleVel
            outMsg.header.stamp = rospy.Time.now()
            outMsg.header.seq = msg.header.seq
            lock.release()

            vectornavOdom.publish(outMsg)

        def sub_imu(msg):
            global outMsg
            global lock
            lock.acquire()
            outMsg.twist.twist.angular.z = msg.angular_velocity.z
            # outMsg.twist.covariance[35] = msg.angular_velocity_covariance[8]
            outMsg.twist.covariance[35] = 0.1 # no covariance in AMZ data
            lock.release()
            # don't publish here

        def sub_gps(msg):
            msg.position_covariance = [
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            vectornavGPS.publish(msg)

        rospy.Subscriber("imu", Imu, sub_imu)
        rospy.Subscriber("gps", NavSatFix, sub_gps)
        rospy.Subscriber("optical_speed_sensor", TwistStamped, sub_optical)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass