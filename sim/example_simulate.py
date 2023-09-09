import numpy as np
import rowan as rn
import matplotlib.pyplot as plt
from uavDy import uav
from uavDy.uav import skew
from Animator import animate 
from trajectoriescsv import *
import time
import argparse
import sys
from itertools import permutations, combinations, chain
from pathlib import Path
# import cffirmware
import yaml
from simulate import * 
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config', type=str, help="Path of the config file")
    args   = parser.parse_args()   
    with open(args.config) as f:
        params = yaml.load(f, Loader=yaml.FullLoader)

    # loads and set all params from the yaml file cfg/rig/2uavs.yaml
    # reference of the paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7843619
    plStSize, uavs, uavs_params, payload, trajectories, pltrajectory = setTeamParams(params)

    input = pltrajectory
    # [t x,y,z, vx,vy, vz, ax,ay,az] reference traj for the payload
    timeStamped_traj = np.loadtxt(input, delimiter=',') 
    tf_ms = timeStamped_traj[0,-1]*1e3
    simtime  = float(params['simtime'])
    tf_sim = tf_ms + simtime
    dt =0.001
    tt = np.linspace(0,tf_sim, num=int(tf_sim/dt))
    # this example is only for 2 uavs: if you want more you will need to import more files
    # cf1 uav state: pos, vel, quat, w
    # load the uav states
    with open("example_2_rig/cf1.csv") as f:
        cf1 = np.loadtxt(f, delimiter=',')
    with open("example_2_rig/cf2.csv") as f:
        cf2 = np.loadtxt(f, delimiter=",")

    # load actions: motor forces divided by u_nominal: m*9.81
    with open("example_2_rig/action_1.csv") as f:
        u1 = np.loadtxt(f, delimiter=',')
    with open("example_2_rig/action_2.csv") as f:
        u2 = np.loadtxt(f, delimiter=",")
    # load the payload states
    with open("example_2_rig/payload.csv") as f:
        payload_state = np.loadtxt(f, delimiter=',')

    cfs = [np.array(cf1), np.array(cf2)]
    u = [np.array(u1), np.array(u2)]
    
    timestep = 4000 # choose the timestep to propagate from
    ctrlInputs = np.zeros((1,4))
    # uncomment this to go through all states and actions
    # for tick, i in enumerate(tt):    
    for id, cf, ui in zip(uavs.keys(), cfs, u):
        uavs[id].state = np.copy(cf[timestep,0:13])
        payload.state = np.copy(payload_state[timestep])
        # get the action and multiply by u_nominal
        control_inp = np.copy(uavs[id].ctrlAll @ ui[timestep]* uavs[id].m * 9.81)
        # stack them
        ctrlInputs  = np.vstack((ctrlInputs, control_inp.reshape(1,4)))
        q = np.copy(uavs[id].state[6:10])
        R = rn.to_matrix(q)
        # this is the u_i in the paper: see paragraph below equation (8) and ignore -ve sign 
        ctrlInp = np.copy(control_inp[0]*R@np.array([0,0,1]))
        payload.stackCtrl(ctrlInp.reshape(1,3))  
    
    # step function
    uavs, loadState =  payload.stateEvolution(ctrlInputs, uavs, uavs_params)  
    
    # compare output of the step function with the loaded data 
    
    print("difference against previous state")
    print(payload_state[timestep] - loadState)
    print(np.linalg.norm( loadState[0:3] - payload_state[timestep][0:3]) )
    print(np.linalg.norm( loadState[3:6] - payload_state[timestep][3:6]) )
    print(np.linalg.norm( loadState[6:10] - payload_state[timestep][6:10]) )
    print(np.linalg.norm( loadState[10:13] -payload_state[timestep][10:13]) )

    print("difference against next state -- this should be zero!!")
    print(payload_state[timestep+1] - loadState)
    print(np.linalg.norm( loadState[0:3] -    payload_state[timestep+1][0:3]) )
    print(np.linalg.norm( loadState[3:6] -   payload_state[timestep+1][3:6]) )
    print(np.linalg.norm( loadState[6:10] -  payload_state[timestep+1][6:10]) )
    print(np.linalg.norm( loadState[10:13] - payload_state[timestep+1][10:13]) )



    # print(loadState[3:6],   payload_state[timestep][3:6])
    # print(loadState[6:10],  payload_state[timestep][6:10])
    # print(loadState[10:13], payload_state[timestep][10:13])
    #

if __name__ == '__main__':
    main()
