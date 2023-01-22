import osqp
import cvxpy as cp
import numpy as np
import scipy.sparse as sp

"""

"""

def main():

        # cvxpy Problem
        x = cp.Variable(name="x")
        y = cp.Variable(name="y")
        prob = cp.Problem(cp.Minimize((x+y)**2),[])
        prob.solve(solver='OSQP', verbose=True)
        data, _, _ = prob.get_problem_data(cp.OSQP)
        print("P", data['P'].todense())
        print(x.value)
        print(y.value)

        # extract matrices
        data, chain, inverse_data = prob.get_problem_data(cp.OSQP)
        # print(data)
        # print(chain)
        print(inverse_data)
        print(inverse_data[2].id2var)
        exit()

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

        # Create an OSQP object
        prob = osqp.OSQP()
        P = sp.csc_matrix([[1,0],[2,1]])
        q = np.array([5,0])
        # A = sp.csc_matrix([])
        # l = np.array([])
        # u = np.array([])
        prob.setup(P, q, A, l, u, alpha=1.0)
        results = prob.solve()
        print(results.x)




if __name__ == '__main__':
    main()