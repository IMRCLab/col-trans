import osqp
import numpy as np
from scipy import sparse
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

def skew(w):
    w = w.reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))

"""
Structure of the QP in OSQP regarding this problem is as follows: 
min (1/2)mu_des'@P@mu_des + q'@mu_des
such that : [Fd                     [Aeq,          [Fd
         np.inf*np.ones(2,)]   <=   Aineq]    <=    np.zeros(2,)]

Where, 
    Aeq = Allocation Matrix (P in the paper)
    Aineq: it was computed numerically in pyCrazyflie code having  20 degrees as limit
            but it can be updated with prob.update(Ax=Ax_new, Ax_idx=Ax_new_idx)
    Fd: is the desired force applied on the payload.
        initially it is set to the wieght of the payload, but it can also be updated with
        prob.update(q=q_new, l=l_new, u=u_new)
"""

# Aineq: Aineq@mu <= np.zeros(2,) --> Hand crafted plane constraints 
# Aineq = sparse.csc_matrix([[ 0.01938242, -0.10992316, 0.04000876, 0, 0, 0],
#                            [ 0, 0, 0, -0.01938242, 0.10992316, 0.04000876]])

Aineq = sparse.csc_matrix([[ -0.15691592, -0.23329555,  0.06665812,  0.        ,  0.        ,  0.         ,  0.        ,  0.        ,  0.        ],
                            [ 0.15691592, -0.23329555,  0.06665812,  0.        ,  0.        ,  0.         ,  0.        ,  0.        ,  0.        ],
                            [ 0.,          0.        ,  0.        , 0.12358678 , 0.25253869 , 0.06665937  ,  0.        ,  0.        ,  0.        ],
                            [ 0.,          0.        ,  0.        , 0.28050056, -0.01924231 ,  0.06665733 ,  0.        ,  0.        ,  0.        ],
                            [ 0.,          0.        ,  0.        ,  0.        ,  0.        ,  0.         ,  -0.12358678,  0.25253869, 0.06665937],
                            [ 0.,          0.        ,  0.        ,  0.        ,  0.        ,  0.         , -0.28050056, -0.01924231,  0.06665733]])

allocMatrix = np.zeros((6, 9)) # AllocMatrix@mu = Fd
attachmentPoints = [
    [-0.041,  0.0355, 1],
    [-0.041, -0.0355, 1],
    [0.041,  -0.0355, 1]
]
k = 0
for i in range(0,9,3):
    allocMatrix[0:3,i:i+3] = np.eye(3)
    allocMatrix[3::,i:i+3] = skew(np.array(attachmentPoints[k]))
    k+=1
Aeq   = allocMatrix
# A matrix
A = sparse.vstack((Aeq, Aineq), format='csc') 
# P matrix
P = sparse.csc_matrix(np.eye(9))
# q vector
q = np.zeros(9)
# lower bound--> [Fd, -np.inf*ones(2,)]
# set Fd to [0,0,0.0981] -->[0,0,mass*gravity]: mass of payload = 0.01 kg, gravity: 9.81 m/s^2
l = np.hstack(([0,0,0.0981, 0, 0, 0], -np.inf*np.ones(6,)))
u = np.hstack(([0,0,0.0981, 0, 0, 0], np.zeros(6,)))

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
