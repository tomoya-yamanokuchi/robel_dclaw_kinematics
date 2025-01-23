import numpy as np
from .. import InverseKinematics


def run_tests():
    inverse = InverseKinematics()

    endeffector_position = np.array(
        [
            [220.664, 0.0, 0.0],
            # [68.5, -151.0, 0.0],
            # [152.1641, -68.5, 0.],
            # [152.1641, -100, 0.],
        ]
    )
    joint_position =  inverse.calc_1claw(endeffector_position)
    # print(joint_position)

    endeffector_position = np.array(
        [
            [220.664, 0.0, 0.0, 220.664, 0.0, 0.0, 220.664, 0.0, 0.0],
        ]
    )
    joint_position =  inverse.calc(endeffector_position)
    print(joint_position)


if __name__ == "__main__":
    run_tests()
