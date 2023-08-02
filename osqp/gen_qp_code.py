import numpy as np
import argparse
import osqp
from scipy import sparse
from scipy.linalg import block_diag
from postprocessing import postprocess
import matplotlib.pyplot as plt
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

def generate_block_diagonal(rows, columns):
    blkrows = int(rows/columns)
    matrix = np.zeros((rows, 3*columns))
    randBlk = np.ones((blkrows,3))
    for i, j in zip(range(0, rows, blkrows), range(0,3*columns,3)):
        matrix[i:i+blkrows, j:j+3] = randBlk
    return matrix

def genOSQPprob(nhps, n, loadType, qpWorkspace_fileName) -> None:
    
    Aineq = sparse.csc_matrix(generate_block_diagonal(nhps, n))
    if loadType == "point":
        Aeq = np.zeros((3, n*3))
        for i in range(0, Aeq.shape[1],3):
            Aeq[0:3, i:i+3] = np.eye(3)
        l = np.hstack((np.ones(3,), -np.inf*np.ones(nhps,)))
        u = np.hstack((np.ones(3,), np.zeros(nhps,)))
    elif loadType == "rigid":
        Aeq = np.ones((6, n*3))
        for i in range(0, Aeq.shape[1],3):
            Aeq[0:3, i:i+3] = np.ones((3,3))
            Aeq[3:6, i:i+3] = np.ones((3,3))
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
    postprocess(qpWorkspace_fileName,  "../crazyflie-firmware/src/lib/osqp/src/osqp/"+ qpWorkspace_fileName +".c")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uavs', type=int, help="num of uavs")
    parser.add_argument("--mass",default="point", type=str, help="point or rigid. If no input, it will assume a point mass")
    args = parser.parse_args()

    n = args.uavs #number of uavs
    nhps = (n)*(n-1) #total number of hps
    loadType = args.mass # load type
    qpWorkspace_fileName = "workspace_"+str(n)+"uav_" + str(int(nhps)) + "hp"

    # generate the OSQP problem
    genOSQPprob(nhps, n, loadType, qpWorkspace_fileName)

    
if __name__ == "__main__":
    main()