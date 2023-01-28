import osqp
import numpy as np
from scipy import sparse
import utils

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
Aineq = sparse.csc_matrix([[ 0.01938242, -0.10992316, 0.04000876, 0, 0, 0],
                           [ 0, 0, 0, -0.01938242, 0.10992316, 0.04000876]])

allocMatrix = np.zeros((3, 6)) # AllocMatrix@mu = Fd
for i in range(0,6,3):
        allocMatrix[0:3,i:i+3] = np.eye(3)

Aeq   = allocMatrix

# A matrix
A = sparse.vstack((Aeq, Aineq), format='csc') 
# P matrix
P = sparse.csc_matrix(np.eye(6))
# q vector
q = np.zeros(6)
# lower bound--> [Fd, -np.inf*ones(2,)]
# set Fd to [0,0,0.0981] -->[0,0,mass*gravity]: mass of payload = 0.01 kg, gravity: 9.81 m/s^2
l = np.hstack(([0,0,0.0981], -np.inf*np.ones(2,)))
u = np.hstack(([0,0,0.0981], np.zeros(2,)))

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

# export
utils.postprocess("2uav_2hp", "../crazyflie-firmware/src/lib/osqp/src/osqp/workspace_2uav_2hp.c")