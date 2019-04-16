#!/usr/bin/env python

import rospy
import time
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from ParticleFilter.ParticleFilter import ParticleFilter
from ParticleFilter.Particle import Particle
from MotionModels.CTRV import ctrvModel

if __name__ == "__main__":
    try:

        rospy.init_node("odometry_node")
        rviz_particle_pub = rospy.Publisher("odometry_visualizer/particles", Marker, queue_size=1)

        #measurement variances
        stdevX = 0.25
        stdevY = 0.25
        stdevV = 0.1
        stdevT = 0.15
        stdevW = 0.05

        # have a point move around a circle with noisy measurements
        state = (1, 0, 0, 0) #x, dx/dt, y, dy/dt

        # particle filter starting conditions
        initialMean = (1, 0, math.pi / 2, 1, 0, 1, 0, 1)
        initialCov = [
            [stdevX * stdevX, 0, 0, 0, 0, 0, 0, 0],
            [0, stdevY * stdevY, 0, 0, 0, 0, 0, 0],
            [0, 0, stdevT * stdevT, 0, 0, 0, 0, 0],
            [0, 0, 0, stdevV * stdevV, 0, 0, 0, 0],
            [0, 0, 0, 0, stdevV * stdevV, 0, 0, 0],
            [0, 0, 0, 0, 0, stdevW * stdevW, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]
        ]

        measCovariance = [[stdevX * stdevX, 0, 0], [0, stdevY * stdevY, 0], [0, 0, stdevT * stdevT]]
        ctrlCovariance = [[stdevV * stdevV, 0], [0, stdevW * stdevW]]

        filter = ParticleFilter(ctrvModel)
        filter.initParticles(initialMean, initialCov)

        def getMeasurement():
            (x, dx, y, dy) = state
            theta = math.atan2(dy, dx)
            return np.random.multivariate_normal((x, y, theta), measCovariance)

        def step(dt, state):
            # we want a circle, so velocity should be perpendicular to position
            (x, dx, y, dy) = state
            return (x + dx * dt, -y, y + dy * dt, x)

        def getControl(state):
            (_, dx, _, dy) = state
            v = math.sqrt(dx * dx + dy * dy)
            return (math.sqrt(dx * dx + dy * dy), 1)

        def particles_publish(): #rviz_marker
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time()
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

            filter.addParticlesToMarker(marker)

            rviz_particle_pub.publish(marker)

        dt = 0.01
        print (", ").join(["xHat", "yHat", "xEst", "yEst", "xAbs", "yAbs"])
        rate = rospy.Rate(1.0 / dt)
        i = 0

        #for i in range(1000):
        while not rospy.is_shutdown():
            control = getControl(state)
            state = step(dt, state)

            measurement = getMeasurement()
            if(i % 50 == 0): #2 times per second
                filter.update(measurement, measCovariance, 0.75)

            filter.predict(control, ctrlCovariance, dt)
            (xHat, yHat, _) = measurement
            (xAbs, _, yAbs, _) = state
            ((xEst, yEst, theta, _, _, _, _, _), _) = filter.meanAndCov()
            particles_publish()
            rate.sleep()
            i = i+1
            # print (", ").join(map(lambda x : str(x), [xHat, yHat, xEst, yEst, xAbs, yAbs]))


    except rospy.ROSInterruptException:
        pass
