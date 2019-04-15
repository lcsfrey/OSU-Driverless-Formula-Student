#!/usr/bin/env python

import rospy

import math
from ParticleFilter import ParticleFilter
from MotionModels.CTRV import ctrvModel
import numpy as np
import utm

# from see_localization.msg import vehicle_pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from see_ego_motion.msg import see_ego_motion_interface
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Point
from tf import transformations

#set to true to use AMZ data
rosbag = False

particleMean = (0, 0, -2 * math.pi / 3, 0, 0, 0, 0, 0)
particleCov = [
    [1, 0, 0, 0, 0, 0, 0, 0], #x
    [0, 1, 0, 0, 0, 0, 0, 0], #y
    [0, 0, 0, 0, 0, 0, 0, 0], #heading
    [0, 0, 0, 0, 0, 0, 0, 0], #tangential velocity
    [0, 0, 0, 0, 0, 0, 0, 0], #normal velocity
    [0, 0, 0, 0, 0, 0, 0, 0], #angular velocity
    [0, 0, 0, 0, 0, 0, 0, 0], #tangential acceleration
    [0, 0, 0, 0, 0, 0, 0, 0] #normal acceleration
]

class OdometryNode:

    def __init__(self):
        self.node = rospy.init_node('see_ego_motion')
        self.odom_pub = rospy.Publisher("see_ego_motion/odom_filtered", see_ego_motion_interface, queue_size=10)
        self.rviz_particle_pub = rospy.Publisher("odometry_visualizer/particles", Marker, queue_size=1)

        if rosbag:
            rospy.Subscriber("gps", NavSatFix, self.sensor_gps_callback)
            rospy.Subscriber("optical_speed_sensor", TwistStamped, self.sensor_odom_callback)
        else:
            #rospy.Subscriber("see_localization/vehicle_pose", vehicle_pose, self.slam_pose_callback)
            rospy.Subscriber("vectornav/GPS", NavSatFix, self.sensor_gps_callback)
            rospy.Subscriber("vectornav/Odom", Odometry, self.sensor_odom_callback)

        self.filter = ParticleFilter.ParticleFilter(ctrvModel, numParticles=20)
        self.filter.initParticles(particleMean, particleCov)
        self.sensorTimeLast = rospy.Time.now()
        self.publishTimeLast = rospy.Time.now()
        self.lastOdom = see_ego_motion_interface() #for visualization hack

        self.odom_pub = rospy.Publisher("see_ego_motion/odom_filtered", see_ego_motion_interface, queue_size=10)
        self.rviz_particle_pub = rospy.Publisher("see_ego_motion_test/odometry_visualization/particles", Marker, queue_size=1)
        self.rviz_odom_pub = rospy.Publisher("see_ego_motion_test/odometry_filtered", Marker, queue_size=1)
        self.rviz_gps_pub = rospy.Publisher("see_ego_motion_test/gps", Marker, queue_size=1)

        #rospy.Subscriber("see_localization/vehicle_pose", vehicle_pose, self.slam_pose_callback)
        rospy.Subscriber("vectornav/GPS", NavSatFix, self.sensor_gps_callback)
        rospy.Subscriber("vectornav/Odom", Odometry, self.sensor_odom_callback)

        self.utm_origin = None

    def gps_marker_publish(self, x, y):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "gps"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration()
        marker.color.a = 1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5

        p = Point()
        p.x = x
        p.y = y
        p.z = 0

        marker.points.append(p)
        self.rviz_gps_pub.publish(marker)

    def odom_marker_publish(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "odom_estimate"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration()
        orientation = transformations.quaternion_from_euler(0, 0, self.lastOdom.theta_global)
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.pose.position.x = self.lastOdom.x_global / 10
        marker.pose.position.y = self.lastOdom.y_global / 10
        marker.pose.position.z = 0
        marker.scale.x = 2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 0.5
        marker.color.b = 0

        self.rviz_odom_pub.publish(marker)

    #odometry_visualizer/particles
    def particles_publish(self): #rviz_marker
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "particle_cloud"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1
        marker.lifetime = rospy.Duration()
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1

        self.filter.addParticlesToMarker(marker)

        self.rviz_particle_pub.publish(marker)

    #see_ego_motion/odom_filtered
    def odom_publish(self):
        (mean, cov) = self.filter.meanAndCov()
        (x, y, t, vt, vn, w, _, _) = mean
        msg = see_ego_motion_interface()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.x_global = x
        msg.y_global = y
        msg.theta_global = t
        msg.velocity_longtitudonal = vt
        msg.velocity_lateral = vn
        msg.yaw_rate = w
        msg.pos_covariance = [
            cov[0][0], cov[0][1],   
            cov[1][0], cov[1][1]
        ]
        msg.theta_variance = cov[2][2]    

        # delta time (to be implemented when a pull request gets approved)
        dt = msg.header.stamp - self.publishTimeLast
        self.publishTimeLast = msg.header.stamp

        self.odom_pub.publish(msg)    
        self.lastOdom = msg

    #vectornav/Odom
    def sensor_odom_callback(self, odomMsg):
        if rosbag:
            twist = odomMsg.twist
            twistCov = [
                0.01, 0, 0, 0, 0, 0,
                0, 0.01, 0, 0, 0, 0,
                0, 0, 0.01, 0, 0, 0, 
                0, 0, 0, 0.01, 0, 0,
                0, 0, 0, 0, 0.01, 0, 
                0, 0, 0, 0, 0, 0.01
            ]
        else:
            twist = odomMsg.twist.twist
            twistCov = odomMsg.twist.covariance

        vel = math.sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y)
        w = twist.angular.z
        control = (vel, w)

        # velocity covariance makes this hard (https://en.wikipedia.org/wiki/Delta_method#Multivariate_delta_method)
        if vel == 0:
            jacobian = np.array([
                [0, 0, 0], 
                [0, 0, 1]
            ])
        else:
            jacobian = np.array([
                [twist.linear.x / vel, twist.linear.y / vel, 0], 
                [0, 0, 1]
            ])

        # just x, y, w covariance
        covarianceTruncated = np.array([
            [twistCov[0], twistCov[1], twistCov[5]],
            [twistCov[6], twistCov[7], twistCov[11]],
            [twistCov[30], twistCov[31], twistCov[35]]
        ])
        # convert to v, w covariance
        ctrlCov = np.matmul(jacobian, np.matmul(covarianceTruncated, np.transpose(jacobian)))

        # delta time
        dt = odomMsg.header.stamp - self.sensorTimeLast
        self.sensorTimeLast = odomMsg.header.stamp
        self.filter.predict(control, ctrlCov, dt.to_sec())

    #vectornav/GPS
    def sensor_gps_callback(self, gpsMsg):
        utm_pos = utm.from_latlon(gpsMsg.latitude, gpsMsg.longitude)
        if not self.utm_origin:
            self.utm_origin = utm_pos

        # assuming earth is 6371 km diameter
        dx = utm_pos[0] - self.utm_origin[0]
        dy = utm_pos[1] - self.utm_origin[1]

        # lack of heading makes this harder
        measurement = (dx, dy, 0)
        if gpsMsg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            measCov = [
                [gpsMsg.position_covariance[0], gpsMsg.position_covariance[1], 0],
                [gpsMsg.position_covariance[3], gpsMsg.position_covariance[4], 0],
                [0, 0, float("inf")] # this should work
            ]
        else:    
            measCov = [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, float("inf")]
            ]

        self.gps_marker_publish(dx / 10, dy / 10)
        self.filter.update(measurement, measCov, 0.8)

    #see_localization/vehicle_pose
    def slam_pose_callback(pose):
        measurement = (pose.x_global, pose.y_global, pose.theta_global)
        # I guess I forgot to ask for covariance
        measurementCovariance = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, math.pi / 6]
        ] # I told SLAM, they are allegedly be working on it
        self.filter.update(measurement, measurementCovariance, 0.75)

if __name__ == "__main__":
    try:
        node = OdometryNode()
        rate = rospy.Rate(10) #because I don't want to kill my computer
        rospy.loginfo("Started see_ego_motion node")
        while not rospy.is_shutdown():
            node.odom_publish()
            node.particles_publish()
            node.odom_marker_publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
