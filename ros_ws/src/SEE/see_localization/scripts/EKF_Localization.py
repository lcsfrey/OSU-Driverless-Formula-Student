#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from see_ego_motion.msg import see_ego_motion_interface
from see_localization.msg import vehicle_pose_msg


state = None #initial state subject to change, haven't figure out how to get this one

cov = np.array([[1,0,0],[0,1,0],[0,0,1.5]])



def motionModel(pre_state,v,w,time):
	_theta = pre_state[2,0]
	_delta_x = -v/w * np.sin(_theta) + v/w * np.sin(_theta + (w * time))
	_delta_y = v/w * np.cos(_theta) - v/w * np.cos(_theta + (w * time))
	_delta_theta = w
	new_state = pre_state + np.array([[_delta_x],[_delta_y],[_delta_theta]])

	return new_state

def Jacobian(stateVector,v,w,time):
	_theta = stateVector[2,0]
	_Jacobian = np.array([[1,0,-v/w * np.cos(_theta) + v/w * np.cos(_theta + (w * time))],
						  [0,1,-v/w * np.sin(_theta) + v/w * np.sin(_theta + (w * time))],
						  [0,0,1]])
	return _Jacobian


def EKF(pre_state,pre_cov,v,w,time,delta_state):
	new_state = motionModel(pre_state,v,w,time)
	F = Jacobian(pre_state,v,w,time)
	new_cov = np.dot(F,pre_cov).dot(F.T) + np.eye(3)

	Q = np.eye(3) #Subject to change

	jH = np.eye(3)

	z_est = delta_state

	y = z_est - new_state

	S = jH.dot(new_cov).dot(jH.T) + Q

	K = new_cov.dot(jH.T).dot(np.linalg.inv(S))

	new_state = new_state + K.dot(y)

	new_cov = (np.eye(new_state.shape[0]) - K.dot(jH)).dot(new_cov)

	return new_state, new_cov




def callback(see_ego_motion_interface):

	v_tan = see_ego_motion_interface.velocity_longtitudonal
	v_nor = see_ego_motion_interface.velocity_lateral
	w = see_ego_motion_interface.yaw_rate

	x_global = see_ego_motion_interface.x_global
	y_global = see_ego_motion_interface.y_global
	theta_global = see_ego_motion_interface.theta_global

	delta_t = see_ego_motion_interface.delta_time

	delta_t = delta_t.to_sec()

	v = np.sqrt(v_tan ** 2 + v_nor ** 2)

	global state
	global cov

	if(state == None): # might have a better way to set initial value
		state = np.array([[x_global],[y_global],[theta_global]])
		rospy.loginfo("Set initial position at:[%s,%s,%s]",state[0,0],state[1,0],state[2,0])
		return

	else:

		rospy.loginfo("----------------------------------------------")
		rospy.loginfo("state before:[%s,%s,%s]",state[0,0],state[1,0],state[2,0])

		delta_state = np.array([[x_global],
								[y_global],
								[theta_global]])
		rospy.loginfo("receving :[%s,%s,%s]",delta_state[0,0],delta_state[1,0],delta_state[2,0])


		state, cov = EKF(state,cov,v,w,delta_t,delta_state)

		rospy.loginfo("state after:[%s,%s,%s]",state[0,0],state[1,0],state[2,0])
		rospy.loginfo("----------------------------------------------")


		covMsg = [cov[0,0],cov[0,1],cov[0,2],cov[1,0],cov[1,1],cov[1,2],cov[2,0],cov[2,1],cov[2,2]]

		pub = rospy.Publisher("/see_localization/Vehicle_pose", vehicle_pose_msg,queue_size=10)
		msg = vehicle_pose_msg()
		msg.x_global = state[0,0]
		msg.y_global = state[1,0]
		msg.theta_global = state[2,0]
		msg.corvariance = covMsg
		pub.publish(msg)



		return


def main():
	rospy.init_node("EKF_localization")
	'''dynamics_sub = message_filters.Subscriber("dynamics", Dynamics)
	delta_sub = message_filters.Subscriber("delta", OdomDeltas)
	delta_t = message_filters.Subscriber("delta_t", see_ego_motion_interface)


	ts = message_filters.TimeSynchronizer([dynamics_sub,delta_sub,delta_t],10)
	ts.registerCallback(callback)'''

	rospy.Subscriber("/see_ego_motion_interface/EKF", see_ego_motion_interface,callback)

	rospy.spin()

	return

if __name__ == '__main__':
	main()