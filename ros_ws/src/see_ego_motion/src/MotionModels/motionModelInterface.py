from abc import ABCMeta, abstractmethod

class IMotionModel:
    __metaclass__ = ABCMeta

    #x(n-1) -> x(n)
    # - particle defined in Particle.py
    # - control has yet to be fully defined
    # - covariance is dependent on control being defined
    # - dt is obvious
    @abstractmethod
    def propogate(self, particle, control, ctrlCovariance, dt):
        raise NotImplementedError