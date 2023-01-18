import osqp
import cvxpy as cp
import numpy as np
import scipy.sparse as sp

from postprocessing import postprocess

"""
Hyperplane for rigid case
"""

def main():

        # cvxpy Problem
        p1a = np.array([1,2,3])  # position of UAV1
        p1b = np.array([4,5,6])  # position of attachment point of UAV1
        p2a = np.array([7,8,9])  # position of UAV2
        p2b = np.array([10,11,12])  # position of attachment point of UAV2
        lambda_svm = 13
        Fd1 = np.array([14,15,16]) # Fd for uav1 (absolute frame)
        Fd2 = np.array([17,18,19]) # fd for uav2 (absolute frame)

        p1a = np.array([-0.000000, 0.392032, 0.521948])
        p2a = np.array([0.000000, -0.392032, 0.521948])
        p1b = np.array([0.000000, 0.300000, 0.000000])
        p2b = np.array([0.000000, -0.300000, 0.000000])
        Fd1 = np.array([-0.000000, 0.300000, 0.056386])
        Fd2 = np.array([-0.000000, -0.300000, 0.056386])
        lambda_svm = 1000.000000

        # n . x = a
        n = cp.Variable(3)
        a = cp.Variable(1)
        prob = cp.Problem(cp.Minimize(cp.sum_squares(n) + lambda_svm * (n.T @ Fd1 - a)**2 + lambda_svm * (n.T @ Fd2 - a)**2),
                [
                n.T @ p1a - a <= -1,
                n.T @ p1b - a <= -1,
                n.T @ p2a - a >= 1,
                n.T @ p2b - a >= 1,
                ])
        print(n.value)

        # extract matrices
        data, chain, inverse_data = prob.get_problem_data(cp.OSQP)
        print(data)

        # x.T P x + q.T x s.t.
        # l <= A x <= u
        P = data['P']
        print("P", P.todense()) # objective
        print("P (update order)")
        for k, row in enumerate(P):
                print(k, row)
        q = data['q']
        print("q", q) # objective

        # see https://github.com/cvxpy/cvxpy/issues/477 and
        # https://github.com/cvxpy/cvxpy/blob/ef3c3ed2627e9ff26b62ca9c771e0f3b0c693905/cvxpy/reductions/solvers/qp_solvers/osqp_qpif.py#L56-L63
        A = sp.vstack([data['A'], data['F']]).tocsc()
        print("A", A.todense())
        print("A (update order)")
        for k in range(A.nnz):
                print(k, A.data[k])
        u = np.concatenate([data['b'], data['G']])
        print("u", u)
        l = np.concatenate([data['b'], -np.inf*np.ones(data['G'].shape)])
        print("l", l)

        # generate the code

        # Create an OSQP object
        prob = osqp.OSQP()

        # Setup workspace and change alpha parameter
        prob.setup(P, q, A, l, u, alpha=1.0)

        prob.solve()

        # prob.codegen("cffirmware_osqp/src/generated",
        #         project_type='Makefile',
        #         parameters='matrices',
        #         python_ext_name='emosqp',
        #         force_rewrite=True,
        #         FLOAT=True,
        #         LONG=False)

        # # export
        # postprocess("hyperplane_rb", "../crazyflie-firmware/src/lib/osqp/src/osqp/workspace_hyperplane_rb.c")


if __name__ == '__main__':
    main()