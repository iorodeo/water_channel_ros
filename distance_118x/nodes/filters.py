import roslib
roslib.load_manifest('distance_118x')
import rospy
import cv

class KalmanFilter(object):

    def __init__(self):
        self.kal = cv.CreateKalman(2,1,0)
        cv.SetIdentity(self.kal.transition_matrix, 1.0)
        cv.SetIdentity(self.kal.measurement_matrix, 1.0)
        cv.SetIdentity(self.kal.process_noise_cov, 10.0)
        self.kal.process_noise_cov[0,0] = 1.0
        self.kal.process_noise_cov[1,1] = 800.0
        cv.SetIdentity(self.kal.measurement_noise_cov, 1.0)
        cv.SetIdentity(self.kal.error_cov_post, 1.0)
        self.measurement = cv.CreateMat(1,1,cv.GetElemType(self.kal.state_pre))
        self.t_previous = None

    def update(self,z,t):
        self.t_current = t
        x, vx = None, None
        if self.update_dt():
            cv.KalmanPredict(self.kal)
            self.measurement[0,0] = z
            state_post = cv.KalmanCorrect(self.kal,self.measurement)
            x = state_post[0,0]
            vx = state_post[1,0]
        return (x,vx)

    def update_dt(self):
        status = False
        if self.t_previous:
            self.dt = self.t_current - self.t_previous
            self.kal.transition_matrix[0,1] = self.dt
            status = True
        self.t_previous = self.t_current
        return status
