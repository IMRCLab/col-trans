import numpy as np
import argparse
import osqp
from scipy import sparse
from scipy.linalg import block_diag
from postprocessing import postprocess
from helper import generate_block_diagonal
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)


def genOSQPprob(nhps, n, loadType, qpWorkspace_fileName) -> None:
    
    Aineq = sparse.csc_matrix(generate_block_diagonal(nhps, n))
    if loadType == "point":
        Aeq = np.ones((3, n*3))
        l = np.hstack((np.ones(3,), -np.inf*np.ones(nhps,)))
        u = np.hstack((np.ones(3,), np.zeros(nhps,)))
    elif loadType == "rigid":
        Aeq = np.ones((6, n*3))
        l = np.hstack((np.ones(6,), -np.inf*np.ones(nhps,)))
        u = np.hstack((np.ones(6,), np.zeros(nhps,)))
    else:
        print("Wrong body type, use point or rigid")
        exit()
    A = sparse.vstack((Aeq, Aineq), format='csc') 
    P = sparse.csc_matrix(np.eye(n*3))
    q = np.zeros(n*3)

    if loadType == "rigid":    
        qpWorkspace_fileName += "_rig"

    prob = osqp.OSQP()
    prob.setup(P, q, A, l, u, alpha=1.0)
    prob.codegen("cffirmware_osqp/src/generated",
        project_type='Makefile',
        parameters='matrices',
        python_ext_name='emosqp',
        force_rewrite=True,
        FLOAT=True,
        LONG=False)
    # "../crazyflie-firmware/src/lib/osqp/src/osqp/"
    postprocess(qpWorkspace_fileName,  qpWorkspace_fileName +".c")

def createCode(n, loadType):
    if loadType == "point":
        
        pass
    elif loadType == "rigid":
        pass
    else:
        pass


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--uavs', type=int, help="num of uavs")
    parser.add_argument("--mass",default="point", type=str, help="point or rigid. If no input, it will assume a point mass")
    args = parser.parse_args()

    n = args.uavs #number of uavs
    nhps = (n)*(n-1) #total number of hps
    loadType = args.mass # load type
    qpWorkspace_fileName = "workspace_"+str(n)+"uav_" + str(int(nhps/2))

    # generate the OSQP problem
    genOSQPprob(nhps, n, loadType, qpWorkspace_fileName)

    filename = qpWorkspace_fileName + '_code' + ".c"
    # generate the text for the code
    # code = createCode(n, loadType)
    # # write the code in file
    # with open(filename, "w") as f:
    #     file.write(code)
    
if __name__ == "__main__":
    main()