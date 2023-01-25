import osqp
import cvxpy as cp
import numpy as np
import scipy.sparse as sp

import utils

"""
Hyperplane for rigid case
"""

def main():

        # cvxpy Problem
        p1 = np.array([1,2,3])  # position of UAV1
        p1a = np.array([4,5,6])  # position of attachment point of UAV1
        p2 = np.array([7,8,9])  # position of UAV2
        p2a = np.array([10,11,12])  # position of attachment point of UAV2
        lambda_svm = 13
        Fd1 = np.array([14,15,16]) # Fd for uav1 (absolute frame)
        Fd2 = np.array([17,18,19]) # fd for uav2 (absolute frame)

        # p1a = np.array([0.1, 0.392032, 0.521948])
        # p2a = np.array([0.1, -0.392032, 0.521948])
        # p1b = np.array([0.1, 0.300000, 0.000000])
        # p2b = np.array([0.1, -0.300000, 0.000000])
        # Fd1 = np.array([0.1, 0.300000, 0.056386])
        # Fd2 = np.array([0.1, -0.300000, 0.056386])
        # lambda_svm = 1000.000000

        # n . x = a
        n = cp.Variable(3)
        a = cp.Variable(1)
        slack1 = cp.Variable(1)
        slack2 = cp.Variable(1)
        prob = cp.Problem(cp.Minimize(cp.sum_squares(n) + lambda_svm * (slack1+slack2)),
                [
                n.T @ p1  - a <= -1,
                n.T @ p1a - a <= -1,
                n.T @ p2  - a >= 1,
                n.T @ p2a - a >= 1,
                n.T @ (p1a+Fd1) - a <= -1+slack1,
                n.T @ (p2a+Fd2) - a >= 1-slack2,
                slack1 >= 0,
                slack2 >= 0,
                ])
        prob.solve()
        print(n.value)
        print(a.value)

        P, q, A, l, u = utils.output_osqp(prob)
        utils.gen_osqp(P, q, A, l, u)

        # export
        utils.postprocess("hyperplane_rb", "../crazyflie-firmware/src/lib/osqp/src/osqp/workspace_hyperplane_rb.c")

        exit()

        # testing
        import emosqp

        # p1 = np.array([0.0, 0.392032, 0.521948])
        # p2 = np.array([0.0, -0.392032, 0.521948])
        # p1_attached = np.array([0.0, 0.300000, 0.000000])
        # p2_attached = np.array([0.0, -0.300000, 0.000000])
        # Fd1t = np.array([0.0, 0.300000, 0.056386])
        # Fd2t = np.array([0.0, -0.300000, 0.056386])
        # lambda_svm = 0.000000

        # Px_new = np.array([2 * lambda_svm, 2 * lambda_svm])
        # Px_new_idx = np.array([3,4], dtype=np.int32)
        # emosqp.update_P(Px_new, Px_new_idx, 2)

        # Ax_new = np.array([
        #         Fd1t[0], Fd2t[0], p1[0], p1_attached[0], -p2[0], -p2_attached[0],
        #         Fd1t[1], Fd2t[1], p1[1], p1_attached[1], -p2[1], -p2_attached[1],
        #         Fd1t[2], Fd2t[2], p1[2], p1_attached[2], -p2[2], -p2_attached[2]])

        # Ax_new_idx = np.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17], dtype=np.int32)
        # emosqp.update_A(Ax_new, Ax_new_idx, 18)

        x, y, status_val, iter, run_time = emosqp.solve()
        print(x)
        print(status_val)
        print(iter)


if __name__ == '__main__':
    main()