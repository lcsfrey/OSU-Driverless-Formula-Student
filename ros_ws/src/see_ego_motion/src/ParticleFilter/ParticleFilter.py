import numpy as np
import math
import Particle
import rospy

class ParticleFilter:
    def __init__(self, motionModel, numParticles=250):
        self.particleSet = []
        self.model = motionModel
        self.numParticles = numParticles

    def initParticles(self, mean, cov):
        newParticles = []
        for i in range(0, self.numParticles):
            newParticles.append(Particle.getInitial(mean, cov))

        self.particleSet = newParticles

    def predict(self, control, ctrlCovariance, dt):
        for particleIndex in range(self.numParticles):
            newParticle = self.particleSet[particleIndex]
            newParticle = self.model.propogate(newParticle, control, ctrlCovariance, dt) #propogate forwards
            self.particleSet[particleIndex] = newParticle

    # add particle weights -> resample (measWeight * numParticles) weighted particles, leave rest alone
    def update(self, measurement, measCovariance, measWeight):
        weights = map(lambda state: Particle.calculateWeight(state, measurement, measCovariance), self.particleSet) #extract weights from particles

        # normalize weights so they sum to 1 (numpy.random.choice won't work if this isn't true)
        weightSum = sum(weights)

        numSampled = int(self.numParticles * measWeight)
        # weighted sample (measWeight * numParticles) particles
        newParticleSet = []

        if weightSum == 0: #all particle weights are 0
            numSampled /= 2 #trust this measurement less
            rospy.loginfo("weights summed to zero for " + str(measurement))
            newMeasCov = [
                [measCovariance[0][0], measCovariance[0][1]],
                [measCovariance[1][0], measCovariance[1][1]]
            ]
            newMeas = [measurement[0], measurement[1]]
            for i in range(numSampled):
                #randomly sample particles and replace positions with a sample from the measurement
                p = self.getRandomParticle(None)
                [nx, ny] = np.random.multivariate_normal(newMeas, newMeasCov)
                newParticleSet.append(Particle.Particle(nx, ny, p.t, p.vt, p.vn, p.w, p.at, p.an))
        else:
            weights = map(lambda weight : weight / weightSum, weights)
            for i in range(numSampled):
                newParticleSet.append(self.getRandomParticle(weights))

        # uniformly randomly sample rest of particles
        for i in range(self.numParticles - numSampled):
            newParticleSet.append(self.getRandomParticle(None))

        self.particleSet = newParticleSet

    def addParticlesToMarker(self, markerTemplate):
        for particle in self.particleSet:
            markerTemplate.points.append(Particle.particleToPoint(particle))

    def meanAndCov(self):
        mean = np.mean(self.particleSet, 0)
        cov = np.cov(np.array(map(np.array, self.particleSet)).T)
        return (mean, cov)

    def getRandomParticle(self, weights):
        pIndex = np.random.choice(range(self.numParticles), p=weights)
        return self.particleSet[pIndex]
