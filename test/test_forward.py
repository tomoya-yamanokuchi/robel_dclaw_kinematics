import numpy as np
from .. import ForwardKinematics


def run_tests():
    forward = ForwardKinematics()

    joint_position = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.0, np.pi*0.5, 0.0],
            [0.0, -np.pi*0.5, 0.0],
            [0.0, 0.0, np.pi*0.5],
            [0.0, 0.0, -np.pi*0.5],
            [np.pi*0.5, 0.0, 0.0],
            # [-np.pi*0.5, 0.0, 0.0], # ダメな角度
            [forward.kinematics.theta0_lb+0.01, 0.0, 0.0], # ダメな角度
        ]
    )

    joint_position = np.array(
        [
            [0.0, -np.pi*0.5 , np.pi*0.5],
        ]
    )
    endeffector_position =  forward.calc_1claw(joint_position)

    joint_position = np.array(
        [
            [0.0, -np.pi*0.5 , np.pi*0.5, 0.0, -np.pi*0.5 , np.pi*0.5, 0.0, -np.pi*0.5 , np.pi*0.5],
        ]
    )
    endeffector_position =  forward.calc(joint_position)
    print(endeffector_position)



if __name__ == "__main__":
    run_tests()
