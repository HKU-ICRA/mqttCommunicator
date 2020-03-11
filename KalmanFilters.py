import numpy as np
from pykalman import KalmanFilter
from pykalman import UnscentedKalmanFilter

class _KalmanFilter_(object):
    
    """Kalman Filter"""
    
    def __init__(self, xinit, yinit):
        
        self.Transition_Matrix = [[1, 0, 1, 0, 0.5, 0],
                                  [0, 1, 0, 1, 0, 0.5],
                                  [0, 0, 1, 0, 1, 0],
                                  [0, 0, 0, 1, 0, 1],
                                  [0, 0, 0, 0, 1, 0],
                                  [0, 0, 0, 0, 0, 1]]

        self.Observation_Matrix = [[1, 0, 0, 0, 0, 0],
                                   [0, 1, 0, 0, 0, 0]]

        self.xinit = xinit
        self.yinit = yinit
        self.vxinit = 0.0
        self.vyinit = 0.0
        self.axinit = 0.0
        self.ayinit = 0.0
        
        self.initstate = [self.xinit, self.yinit, self.vxinit,
                          self.vyinit, self.axinit, self.ayinit]
        
        self.initcovariance = 1.0e-3 * np.eye(6)
        self.transistionCov = 1.0e-4 * np.eye(6)
        self.observationCov = 1.0e-1 * np.eye(2)
        
        self.mean = None
        self.covariance = None
        
        self.kf = KalmanFilter(transition_matrices = self.Transition_Matrix,
                               observation_matrices = self.Observation_Matrix,
                               initial_state_mean = self.initstate,
                               initial_state_covariance = self.initcovariance,
                               transition_covariance = self.transistionCov,
                               observation_covariance = self.observationCov)
    
    def _filterUpdate_(self, new_x_measurement, new_y_meansurement):
        
        
        if (self.mean is None) or (self.covariance is None):
            self.mean, self.covariance = self.kf.filter_update(self.initstate,
                                                               self.initcovariance,
                                                               [new_x_measurement,
                                                                new_y_meansurement])
        else:
            self.mean, self.covariance = self.kf.filter_update(self.mean,
                                                               self.covariance,
                                                               [new_x_measurement,
                                                                new_y_meansurement])
        
        return self.mean[0], self.mean[1]
        
