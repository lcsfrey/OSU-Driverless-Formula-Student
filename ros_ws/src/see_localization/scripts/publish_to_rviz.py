#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from see_ego_motion.msg import see_ego_motion_interface
from see_localization.msg import vehicle_pose_msg
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations as tf

def circle_points(r, n):
    circles = []
    for r, n in zip(r, n):
        t = np.linspace(0, 2*np.pi, n)
        x = r * np.cos(t) - 25
        y = r * np.sin(t)
        temp  = t + (np.pi/2)
        circles.append(np.c_[x, y, temp])
    return circles

def callback(vehicle_pose_msg):
	x_position = vehicle_pose_msg.x_global
	y_position = vehicle_pose_msg.y_global
	theta = vehicle_pose_msg.theta_global
	pub_marker = rospy.Publisher("Map", MarkerArray, queue_size=10)
	markerArray = MarkerArray()

	n = [50,50,100]
	r = [20,30,25]
	circles = circle_points(r, n)
	_map=[]
	_drivingLine=[]
	counter = 0
	for point in circles[0]:
		marker = Marker()
		marker.header.frame_id = "EKF"
		marker.header.stamp = rospy.Time()
		marker.ns = "inner"
		marker.type = marker.SPHERE
		marker.action = 0
		marker.id = counter
		marker.pose.position.x = point[0]
		marker.pose.position.y = point[1]
		marker.pose.position.z = 0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.5
		markerArray.markers.append(marker)
		counter += 1


	counter = 0
	for point in circles[1]:
		marker = Marker()
		marker.header.frame_id = "EKF"
		marker.header.stamp = rospy.Time()
		marker.ns = "outter"
		marker.type = marker.SPHERE
		marker.action = 0
		marker.id = counter
		marker.pose.position.x = point[0]
		marker.pose.position.y = point[1]
		marker.pose.position.z = 0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.5
		markerArray.markers.append(marker)
		counter += 1

	counter = 0	
	for point in circles[2]:
		marker = Marker()
		marker.header.frame_id = "EKF"
		marker.header.stamp = rospy.Time()
		marker.ns = "path"
		marker.type = marker.SPHERE
		marker.action = 0
		marker.id = counter
		marker.pose.position.x = point[0]
		marker.pose.position.y = point[1]
		marker.pose.position.z = 0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.5
		markerArray.markers.append(marker)
		counter += 1

	pub_marker.publish(markerArray)
		

	quaternion = tf.quaternion_from_euler(0, 0, theta)
	pub = rospy.Publisher("RvizPose", PoseStamped,queue_size=10)
	msg = PoseStamped()
	msg.header.frame_id = "EKF"
	msg.header.stamp = rospy.Time()
	msg.pose.position.x = x_position
	msg.pose.position.y = y_position
	msg.pose.position.z = 0
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	pub.publish(msg)
	



def main():
	rospy.init_node("RvizData")
	'''dynamics_sub = message_filters.Subscriber("dynamics", Dynamics)
	delta_sub = message_filters.Subscriber("delta", OdomDeltas)
	delta_t = message_filters.Subscriber("delta_t", see_ego_motion_interface)


	ts = message_filters.TimeSynchronizer([dynamics_sub,delta_sub,delta_t],10)
	ts.registerCallback(callback)'''

	rospy.Subscriber("/see_localization/Vehicle_pose", vehicle_pose_msg,callback)

	rospy.spin()

	return

if __name__ == '__main__':
	main()