import numpy as np
from .utils import KinematicsDefinition


class ForwardKinematics:
    def __init__(self):
        self.num_joint  = 9
        self.kinematics = KinematicsDefinition()


    def calc(self, theta) -> np.ndarray:
        '''
        input:
            joint position (1 claw)
                shape = (data_num, 9)
        return:
            end-effector position: size (N, 9)
        '''
        # -----------
        num_shape = len(theta.shape)
        if num_shape == 1:
            assert theta.shape == (self.num_joint,)
            theta = theta.reshape(1, self.num_joint)
        else:
            assert theta.shape[-1] == self.num_joint
        # -----------
        pos = [0]*3
        for i, theta_1claw in enumerate(np.split(theta, 3, axis=-1)):
            pos[i] = self.calc_1claw(theta_1claw)
        return np.concatenate(pos, axis=-1)


    def calc_1claw(self, theta):
        '''
            theta : size (N, dim_theta)
        '''
        self.kinematics.check_feasibility(theta)
        # ----
        theta0 = theta[:, 0]
        theta1 = theta[:, 1]
        theta2 = theta[:, 2]
        # ----
        L = self.kinematics.l0 + (self.kinematics.l1 * np.cos(theta1)) + (self.kinematics.l2 * np.cos(theta1 + theta2))
        px = L * np.cos(theta0)
        py = (self.kinematics.l1 * np.sin(theta1)) + (self.kinematics.l2 * np.sin(theta1 + theta2))
        pz = L * np.sin(theta0)
         # ----
        # import ipdb; ipdb.set_trace()
        return np.c_[px, py, pz]


    def get_cog(self, pos):
        return np.mean(pos, axis=0)




