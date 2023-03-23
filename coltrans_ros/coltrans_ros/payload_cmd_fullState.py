
import numpy as np
from pathlib import Path
import yaml
import rowan as rn
from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory

FREQUENCY = 50 #Hz
TIMESCALE = 1.2
HEIGHT = 0.75
LOGGING = False
IDs = [4,7]

def derivative(vec, dt):
    dvec  =[]
    for i in range(len(vec)-1):
        dvectmp = (vec[i+1]-vec[i])/dt
        dvec.append(dvectmp)
    return np.asarray(dvec)


## from : https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L904
def qinv(q):
    return rn.conjugate(q)

## from: https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L896
def qqmul(q, p):
    return rn.multiply(q, p)

## from: https://github.com/IMRCLab/crazyflie-firmware/blob/17ccb8e553d0dba0294ca4bc5e159ed279724814/src/modules/interface/math3d.h#L989
def quat2omega(quat,dt):
    omega = []
    for i in range(len(quat)-1):
        q0 = quat[i]
        q1 = quat[i+1]
        q_dot =[
            (q1[0] - q0[0]) / dt,
            (q1[1] - q0[1]) / dt,
            (q1[2] - q0[2]) / dt,
            (q1[3] - q0[3]) / dt]
        
        q_inv = qinv(q0);
        r = qqmul(q_dot, q_inv)
        w = 2*r[1::]
        omega.append(w.tolist())
    return np.asarray(omega);

def polartovector(cablestate):
    # returns the points to be visualized for the cable 
    az = cablestate[0]
    el = cablestate[1]
    # azimuth and elevation --> unit vec
    # source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    unitvec = [np.sin(az)*np.sin(el),
               np.cos(az)*np.sin(el),
               np.cos(el)]
    return unitvec



def executeTrajectory(timeHelper, allcfs, position, velocity, cableAngleswithIDs, rate=100, offset=np.zeros(3)):
    
    start_time = timeHelper.time()
    i = 0
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time

        if i >= len(velocity):
            break
        pos = position[i]
        vel = velocity[i]
        allcfs.cmdFullState(
            pos + offset,
            vel,
            np.zeros_like(vel),
            0,
            np.zeros_like(vel))
        
        allcfs.cmdDesCableAngles(cableAngleswithIDs[i])
        i+=1
        timeHelper.sleepForRate(rate)
def main():

    # parser = argparse.ArgumentParser()
    # parser.add_argument("motions", type=str, help="output file containing solution")
    # args = parser.parse_args()
    # motions_file_path = args.motions
    motions_file_path = "/home/khaledwahba94/imrc/col-trans/coltrans_ros/data/2cfs_pointmass_output.yaml"
    with open(motions_file_path) as motions_file:
        motions = yaml.load(motions_file, Loader=yaml.FullLoader)

    dt = 1/FREQUENCY
    states = motions["result"][0]["states"]
    
    # payload postion
    position = np.asarray([state[0:3] for state in states])

    # velocity postion
    velocity = derivative(position, dt)

    # acceleration postion
    acceleration = derivative(velocity, dt)
    
    #time array
    time = [0]
    for i in range(len(position)-1):
        time.append(time[i]+dt)

    # payload orientation
    quat     = [state[3:7] for state in states]
    euler    = rn.to_euler(rn.normalize(quat))
   
   # payload omega
    omega    = quat2omega(quat, dt)

    #cables unit vec
    cables   = np.asarray([state[7::] for state in states])
    num_cables = int(len(cables[0])/2)
    cableAngleswithIDs = []
    for cable in cables:
        data_tmp = []
        for i in range(num_cables):
            az_el = cable[0+2*i: 2+2*i]
            data_tmp.append((IDs[i],  az_el[0], az_el[1]))
        cableAngleswithIDs.append(data_tmp)

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.setParam('stabilizer.controller', 7)
    timeHelper.sleep(2.0)
    allcfs.takeoff(targetHeight=HEIGHT, duration=3.0)
    # timeHelper.sleep(3.0)
    # timeHelper.sleep(5.0) # extra time


    allcfs.emergency()
    rate = FREQUENCY
    print("executing traj")
    executeTrajectory(timeHelper, allcfs, position, velocity, cableAngleswithIDs, offset=np.array([0, 0, HEIGHT]))

    # cf.notifySetpointsStop()
    allcfs.land(targetHeight=0.03, duration=1.5)
    timeHelper.sleep(2.0)

if __name__ == "__main__":
    main()
