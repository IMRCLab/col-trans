import osqp
import cvxpy as cp
import numpy as np
import scipy.sparse as sp

"""

"""

def main():

        # cvxpy Problem
        p1 = np.array([1,2,3])
        p2 = np.array([4,5,6])
        lambda_svm = 13
        Fd = np.array([7,8,9])

        n = cp.Variable(3)
        prob = cp.Problem(cp.Minimize(cp.sum_squares(n) + lambda_svm * (n.T @ Fd)**2 ),
                    [
                        n.T @ p1 <= -1,
                        n.T @ p2 >= 1,
                    ])
        prob.solve()
        print(n.value)

        # extract matrices
        data, chain, inverse_data = prob.get_problem_data(cp.OSQP)
        print(data)

        # x.T P x + q.T x s.t.
        # l <= A x <= u
        P = data['P']
        print("P", P.todense()) # objective
        print("P (update order)\n", P)
        q = data['q']
        print("q", q) # objective

        # see https://github.com/cvxpy/cvxpy/issues/477 and
        # https://github.com/cvxpy/cvxpy/blob/ef3c3ed2627e9ff26b62ca9c771e0f3b0c693905/cvxpy/reductions/solvers/qp_solvers/osqp_qpif.py#L56-L63
        A = sp.vstack([data['A'], data['F']]).tocsc()
        print("A", A.todense())
        print("A (update order)\n", A)
        u = np.concatenate([data['b'], data['G']])
        print("u", u)
        l = np.concatenate([data['b'], -np.inf*np.ones(data['G'].shape)])
        print("l", l)

        # generate the code

        # Create an OSQP object
        prob = osqp.OSQP()

        # Setup workspace and change alpha parameter
        prob.setup(P, q, A, l, u, alpha=1.0)

        prob.codegen("cffirmware_osqp/src/generated",
                project_type='Makefile',
                parameters='matrices',
                python_ext_name='emosqp',
                force_rewrite=True,
                FLOAT=True,
                LONG=False)

        # Solve problem
        res = prob.solve()


if __name__ == '__main__':
    main()