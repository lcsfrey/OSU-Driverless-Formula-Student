#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from see_ego_motion.msg import see_ego_motion_interface
from see_localization.msg import vehicle_pose_msg


def circle_points(r, n):
    circles = []
    for r, n in zip(r, n):
        t = np.linspace(0, 2*np.pi, n)
        x = r * np.cos(t) - 25
        y = r * np.sin(t)
        temp  = t + (np.pi/2)
        circles.append(np.c_[x, y, temp])
    return circles

def Maps():
	n = [50,50,100]
	r = [20,30,25]
	circles = circle_points(r, n)
	_map=[]
	_drivingLine=[]
	for point in circles[0]:
		temp = {"x":point[0],"y":point[1],"mark":1}
		_map.append(temp)
	for point in circles[1]:
		temp = {"x":point[0],"y":point[1],"mark":0}
		_map.append(temp)
	for point in circles[2]:
		temp = {"x":point[0],"y":point[1],"theta": point[2]}
		_drivingLine.append(temp)

	return _map, _drivingLine

def odem(location):
	x=location["x"]
	y=location["y"]
	theta=location["theta"]
	noise_X = np.random.normal(0,0.5)
	noise_Y = np.random.normal(0,0.5)
	noise_T = np.random.normal(0,0.01)

	temp = np.array([[x+noise_X],[y+noise_Y],[theta+noise_T]])
	return temp

def main():
	pub = rospy.Publisher("/see_ego_motion_interface/EKF", see_ego_motion_interface,queue_size=10)

	rospy.init_node("EKF_sample_data")

	globalMap, carLocation = Maps()
	start = 0;
	v = (50*np.pi)/99
	w = (2*np.pi)/9

	for location in carLocation: # for every location, detect cones and add noise to the current location
								# apply EKF to localise the car
								# assume the time between two points is 

	
		odem_location = odem(location)
		gps_x = odem_location[0,0]
		gps_y = odem_location[1,0]
		gps_t = odem_location[2,0]
		v_x = v * np.cos(location["theta"])
		v_y = v * np.sin(location["theta"])
		msg = see_ego_motion_interface()
		msg.velocity_longtitudonal = v_x
		msg.velocity_lateral = v_y
		msg.yaw_rate = w
		msg.x_global = gps_x
		msg.y_global = gps_y
		msg.theta_global = gps_t
		msg.delta_time = rospy.Duration(1, 0)
		pub.publish(msg)
		rospy.loginfo("true location:[%s,%s,%s]",location["x"], location["y"], location["theta"])

		rospy.sleep(1.)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass



