import numpy as np
import rowan
import osqp
import argparse
from scipy import sparse
import pathlib as path

np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

## always assume it is a rigid case
def main(numCables, runs):
    run_time = 0
    numofhps = numCables*(numCables-1)
    Aineq = np.zeros((numofhps, 3*numCables))
    size = Aineq.shape

    numofhpsperCable = int(numofhps/numCables)
    for i, j in zip(range(0, size[0], numofhpsperCable), range(0, numCables*3, 3)):
        Aineq[i:i+numofhpsperCable, j:j+3] = np.random.rand(numofhpsperCable,3)

    Aineq = sparse.csc_matrix(Aineq)
    P = np.ones((6, numCables*3))
    Aeq = P

    A = sparse.vstack((Aeq, Aineq))
    W = sparse.csc_matrix(np.eye(numCables*3))

    # q vector
    q = np.zeros(numCables*3)
    # lower and upper bounds

    l = np.hstack((np.ones(6,),   -np.inf*np.ones(size[0],)))
    u = np.hstack((2*np.ones(6,), np.zeros(size[0],)))

    # Create an OSQP object
    prob = osqp.OSQP()
    
    # Setup workspace and change alpha parameter
    prob.setup(P=W, q=q, A=A, l=l, u=u)

    prob.codegen("src/generated",
        project_type='Makefile',
        parameters='matrices',
        python_ext_name='emosqp',
        force_rewrite=True,
        FLOAT=True,
        LONG=False)

    # testing 
    import emosqp
    for run in range(runs):
        x, y, status_val, iter, run_time_i = emosqp.solve()
        # print("Number of cables: ", numCables)
        # print('status_val: ',status_val)
        # print('iteration: ',iter)
        # print("run time: ", run_time)
        run_time += run_time_i
    run_time_av = run_time/runs

    valuesFilePath = path.Path('./values.txt')

    if not (valuesFilePath.is_file()): 
        with open('values.txt', 'w') as file:
            # file.write('numofcables: '+str(numCables)+' , runtime average: '+str(run_time_av)+'\n'+ ' last sol: '+ str(x))
            file.write(str(numCables)+'  '+str(run_time_av))
    else:
        file = open("values.txt", "a")
        # file.write('\nnumofcables: '+str(numCables)+' , runtime average: '+str(run_time_av)+'\n'+ ' last sol: '+ str(x))
        file.write('\n'+str(numCables)+'  '+str(run_time_av))
        file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--numCables', type=int, help="numberofcables")
    parser.add_argument('--runs', type=int, help="numberofcables")
    args   = parser.parse_args()   
    main(args.numCables, args.runs)


