import numpy as np
import math
from collections import namedtuple
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point

Particle = namedtuple("Particle", ["x", "y", "t", "vt", "vn", "w", "at", "an"])

# for mean, covariance
def getZero():
    return Particle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# for initializing set
def getInitial(mean, cov):
    (x, y, t, vt, vn, w, at, an) = np.random.multivariate_normal(mean, cov)
    return Particle(x, y, t, vt, vn, w, at, an)

# for utility within ParticleFilter
def particleToNp(particle):
    return np.array(particle)

def npToParticle(array):
    return Particle(array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8])

# I had something in mind when I made this, now I can't remember
def sensorsToParticle(odomMsg, accelMsg):
    pose = odomMsg.pose.pose
    twist = odomMsg.twist.twist

    # position, angular velocity are easy
    x = pose.position.x
    y = pose.position.y
    w = twist.angular.z

    # heading requires a bit of math
    quatList = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    (_, _, t) = euler_from_quaternion(quatList)

    # velocity is a bit harder
    headingVector = np.array([cos(t), sin(t)])
    vMagnitudeSqr = twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y
    vTan = np.dot(np.array([twist.linear.x, twist.linear.y]), headingVector)
    vNorm = math.sqrt(vMagnitudeSqr - vTan * vTan)

    # acceleration from IMU
    aMagnitudeSqr = accelMsg.linear_acceleration.x * accelMsg.linear_acceleration.x + accelMsg.linear_acceleration.y * accelMsg.linear_acceleration.y
    aTan = np.dot(np.array([accelMsg.linear_acceleration.x, accelMsg.linear_acceleration.y]), headingVector)
    aNorm = math.sqrt(aMagnitudeSqr - aTan * aTan)

    return Particle(x, y, t, vTan, vNorm, w, aTan, aNorm)

# for weighting, resampling
def particleToMeasurement(particle):
    return (particle.x, particle.y, particle.t)

# for visualization
def particleToPoint(particle):
    p = Point()
    # scaled down for easier viewing in rviz
    p.x = particle.x / 10
    p.y = particle.y / 10
    p.z = 0
    return p

def calculateWeight(particle, measurement, measCovariance):
    stateVector = (particle.x, particle.y, particle.t)
    stateDiff = np.array(stateVector) - np.array(measurement)

    # stupid wrapping radians
    stateDiff[2] = stateDiff[2] % math.pi * 2
    if(stateDiff[2] > math.pi):
        stateDiff[2] = -(math.pi * 2 - stateDiff[2])
    
    stateDiffT = np.transpose(stateDiff)
    covInverse = np.linalg.inv(measCovariance)

    return math.exp(-0.5 * np.matmul(stateDiffT, np.matmul(covInverse, stateDiff)))