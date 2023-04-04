import numpy as np
import rowan
import osqp
import argparse
from scipy import sparse

np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

## always assume it is a rigid case
def main():
    cables = [4, 5, 6, 7, 8]
    for numCables in cables:
        numofhps = numCables*(numCables-1)
        Aineq = np.zeros((numofhps, 3*numCables))
        size = Aineq.shape

        numofhpsperCable = int(numofhps/numCables)
        for i, j in zip(range(0, size[0], numofhpsperCable), range(0, numCables*3, 3)):
            Aineq[i:i+numofhpsperCable, j:j+3] = np.asarray([np.random.uniform(1, 2), np.random.uniform(3, 4), np.random.uniform(4, 5)])

        Aineq = sparse.csc_matrix(Aineq)
        
        P = np.ones((6, numCables*3))
        Aeq = P

        A = sparse.vstack((Aeq, Aineq))
        W = sparse.csc_matrix(np.eye(9))
        # q vector
        q = np.zeros(9)
        # lower and upper bounds
        l = np.hstack((np.ones(6,),   -np.inf*np.ones(size[0],)))
        u = np.hstack((2*np.ones(6,), np.zeros(size[0],)))

        # Create an OSQP object
        prob = osqp.OSQP()
        
        # Setup workspace and change alpha parameter
        prob.setup(W, q, A, l, u)
        # x = prob.solve()

        prob.codegen("src/generated",
            project_type='Makefile',
            parameters='matrices',
            python_ext_name='emosqp',
            force_rewrite=True,
            FLOAT=True,
            LONG=False)

        # testing 
        import emosqp

        x, y, status_val, iter, run_time = emosqp.solve()
        print("Number of cables: ", numCables)
        print("sol: ", x)
        print(status_val)
        print(iter)
        print("run time: ", run_time)

if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument('numCables', type=int, help="numberofcables")
    # args   = parser.parse_args()   
    # main(args.numCables)
    main()


