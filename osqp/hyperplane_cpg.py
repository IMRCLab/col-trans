from cvxpygen import cpg
import cvxpy as cp
import numpy as np


def main():

        # cvxpy Problem
        p1 = cp.Parameter(3, name='p1')
        p2 = cp.Parameter(3, name='p2')
        # lambda_svm = cp.Parameter(1, name='lambda_svm', nonneg=True)
        # Fd = cp.Parameter(3, name='Fd')

        n = cp.Variable(3)
        problem = cp.Problem(cp.Minimize(cp.sum_squares(n)),# + lambda_svm * cp.square(n.T @ Fd)),
                    [
                        n.T @ p1 <= -1,
                        n.T @ p2 >= 1,
                    ])

        p1.value = np.array([1,2,3])
        p2.value = np.array([4,5,6])
        print(problem.solve())
        print(n.value)

        cpg.generate_code(problem, code_dir='hyperplane')

if __name__ == '__main__':
    main()