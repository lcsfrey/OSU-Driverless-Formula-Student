import math
import numpy as np
from ParticleFilter.Particle import Particle
from collections import namedtuple
from motionModelInterface import IMotionModel
from geometry_msgs.msg import Point

## function PropCTRV : propagates a state vector using a CTRV motion model
# Parameters:
#   velocity : The current forward velocity estimate in meters/second
#   turn_rate: The current yaw rate estimate in radians/second
#   time_elapsed : The time to propagate forward in seconds
# Returns:
#   The propagated state vector
def propCTRV(velocity, turn_rate, time_elapsed):
  x = (velocity / turn_rate) * math.sin(turn_rate * time_elapsed)
  y = (velocity / turn_rate) * (1 - math.cos(turn_rate * time_elapsed))
  theta = turn_rate * time_elapsed
  v = velocity
  omega = turn_rate

  return (x, y, theta, v, omega)

## function odomTransform : translates x, y, theta displacement to odom delta model
# Parameters:
#   x_offset : The forward translation in meters
#   y_offset : The lateral translation in meters
#   theta_offset : The offset of the final position angle in radians
# Returns:
#   An odom model representation of the translation and rotation
def odomTransform(x_offset, y_offset, theta_offset):
  delta_trans = math.sqrt(x_offset ^ 2 + y_offset ^ 2)
  delta_rot1 = math.atan(x_offset / y_offset)
  delta_rot2 = theta_offset - delta_rot1

  return (delta_trans, delta_rot1, delta_rot2)

## function odomSample : generates an odom particle by sampling from a distribution
# Parameters:
# Returns:
#   A odom model particle sampled from the distribution
def odomSample(delta_rot1, delta_trans, delta_rot2, a1, a2, a3, a4):
  delta_trans_hat = delta_trans + sampleNorm(a3 * delta_trans + a4 * (math.abs(delta_rot1) + math.abs(delta_rot2)))
  delta_rot1_hat = delta_rot1 + sampleNorm(a1 * math.abs(delta_rot1) + a2 * delta_trans)
  delta_rot2_hat = delta_rot2 + sampleNorm(a1 * math.abs(delta_rot2) + a2 * delta_trans)

  return (delta_trans_hat, delta_rot1_hat, delta_rot2_hat)

class CTRVModel(IMotionModel):

  def propogate(self, particle, control, ctrlCovariance, dt):
    (vHat, wHat) = np.random.multivariate_normal(control, ctrlCovariance)
    (x, y, theta, v, w) = propCTRV(vHat, wHat, dt)
    
    stateOld = (particle.x, particle.y, particle.t)

    #"delta" because propCtrv assumes starting at 0
    stateDelta = (x, y, theta)
    cosTh = math.cos(particle.t)
    sinTh = math.sin(particle.t)

    #old -> new
    transitionMatrix = [
      [cosTh, -sinTh, 0], 
      [sinTh, cosTh, 0], 
      [0, 0, 1]
    ]
    (xNew, yNew, thNew) = np.matmul(transitionMatrix, stateDelta) + stateOld

    # stupid wrapping radians
    thNew = thNew % (math.pi * 2)
    if(thNew > math.pi):
        thNew = -(math.pi * 2 - thNew)

    # handling of vn, w, at, an may need to be changed in the future
    return Particle(xNew, yNew, thNew, particle.vt, particle.vn, particle.w, particle.at, particle.an)

#singleton class
ctrvModel = CTRVModel()