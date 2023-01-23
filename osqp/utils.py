import osqp
import cvxpy as cp
import numpy as np
import scipy.sparse as sp


def output_osqp(prob):
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

def postprocess(name, output, input = "cffirmware_osqp/src/generated/src/osqp/workspace.c"):
    """ Adds static keyword, so the workspace can be re-used """
        
    with open(input) as f:
        contents = f.read()

    datatypes = ["c_int", "c_float", "csc", "OSQPData", "OSQPSettings", "OSQPScaling", "QDLDL_float", "QDLDL_int", "QDLDL_bool", "qdldl_solver", "OSQPSolution", "OSQPInfo"]

    for datatype in datatypes:
        contents = contents.replace("{} ".format(datatype), "static {} ".format(datatype))

    contents = contents.replace("OSQPWorkspace workspace", "OSQPWorkspace workspace_{}".format(name))

    with open(output, 'w') as f:
        f.write(contents)
