import osqp
import cvxpy as cp
import numpy as np
import scipy.sparse as sp

import utils

"""
Fd pair (22) in Lee's paper
"""


def skew(w):
    w = np.asarray(w).reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))


def main():


   # see (22) in Lee's paper
    Fd1 = cp.Variable(3)
    Fd2 = cp.Variable(3)
    p1a_skew = cp.Parameter((3,3), name='p1a_skew')
    p2a_skew = cp.Parameter((3,3), name='p2a_skew')
    RpT = cp.Parameter((3,3), name='RpT')
    Md = cp.Parameter(3, name='Md')
    Fd = cp.Parameter(3, name='Fd')
    prob = cp.Problem(cp.Minimize(
        cp.sum_squares(p1a_skew @ RpT @ Fd1 + p2a_skew @ RpT @ Fd2 - Md)),
            [
                Fd1 + Fd2 == Fd,
                # skew(p1a) @ Rp.T @ Fd1 + skew(p2a) @ Rp.T @ Fd2 == Md,
            ])

    p1a = np.array([1,2,3])
    p2a = np.array([4,5,6])

    p1a_skew.value = skew(p1a)
    p2a_skew.value = skew(p2a)
    Fd.value = np.array([7,8,9])
    Md.value = np.array([10,11,12])
    RpT.value = np.array([[13,14,15],[16,17,18],[19,20,22]])

    utils.output_osqp(prob)

    print(p2a_skew.value @ RpT.value)


if __name__ == '__main__':
    main()