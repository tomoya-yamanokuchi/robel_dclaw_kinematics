import numpy as np
from .utils import KinematicsDefinition


'''
＊注意＊
    ブロックはめ合わせタスクに必要な範囲の値では，解の一意性が問題になっていないが，
    指先が外側を向くような場合などには対応できていないため，タスクを変更する時には
    解の一意性を保証できるように修正する必要がある．
'''


class InverseKinematics:
    def __init__(self):
        self.kinematics = KinematicsDefinition()


    def calc(self, position):
        '''
        input:
            end-effector position: size (N, 9)
        return:
            joint position (1 claw)
                shape = (data_num, 9)
        '''
        theta = [0]*3
        for i, pos_1claw in enumerate(np.split(position, 3, axis=-1)):
            theta[i] = self.calc_1claw(pos_1claw)
        return np.concatenate(theta, axis=-1)


    def calc_1claw(self, position):
        '''
        input:
            endeffector position: size (N, 3)
        return:
            joint position (1 claw)
                shape = (data_num, 3)
                3     = theta_dim
        '''
        assert len(position.shape) == 2
        assert position.shape[-1] == 3
        # ----
        x = position[:, 0]
        y = position[:, 1]
        z = position[:, 2]
        # ----
        theta0 = np.arctan2(z, x) # np.arctan2(y, x)
        # ----
        l      = np.sqrt(x**2 + z**2)
        s      = np.sqrt((l - self.kinematics.l0)**2 + y**2)
        beta   = np.arccos((self.kinematics.l1**2 - self.kinematics.l2**2 + s**2) / (2 * self.kinematics.l1 * s))
        gamma  = np.arctan2(y, l - self.kinematics.l0)
        theta1 = gamma - beta
        # ----
        theta2 = np.pi - np.arccos((self.kinematics.l1**2 + self.kinematics.l2**2 - s**2 ) / (2 * self.kinematics.l1 * self.kinematics.l2))
        # ----
        theta = np.c_[theta0, theta1, theta2]
        self.kinematics.check_feasibility(theta)
        # ----
        return theta



