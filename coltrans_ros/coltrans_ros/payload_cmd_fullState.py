
import numpy as np
from pathlib import Path
import yaml
import rowan as rn
from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory

# FREQUENCY = 85 #Hz
# TIMESCALE = 1.2
# HEIGHT = 0.5
LOGGING = True
IDs = [7, 9]

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



def executeTrajectory(timeHelper, allcfs, position, velocity, quats, omegas, cableAngleswithIDs, rate=100, offset=np.zeros(3), repeat_last_setpoint=0):
    
    start_time = timeHelper.time()
    i = 0
    
    while not timeHelper.isShutdown():

        if i >= len(velocity) + repeat_last_setpoint:
            break
        idx = min(i, len(velocity)-1)
        pos = position[idx]
        vel = velocity[idx]
        quat = quats[idx]
        omega = omegas[idx]

        allcfs.cmdFullState(
            pos + offset,
            vel,
            np.zeros_like(vel),
            quat,
            np.zeros_like(vel))
        allcfs.cmdDesCableAngles(cableAngleswithIDs[idx])
        i+=1
        timeHelper.sleepForRate(rate)
    print("Trajectory executed")
def main():

    # parser = argparse.ArgumentParser()
    # parser.add_argument("motions", type=str, help="output file containing solution")
    # args = parser.parse_args()
    # motions_file_path = args.motions

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
 
    allcfs.setParam('stabilizer.controller', 7)
    allcfs.setParam('ctrlLeeP.form_ctrl', 3)
    # allcfs.emergency()

    if LOGGING:
        print('Logging..')
        allcfs.setParam("usd.logging", 1)
    timeHelper.sleep(3.0)

    traj_counter = 0
    # (filename, rate, repeat_last_setpoint)
    motions_file_paths = [("/home/whoenig/projects/tuberlin/col-trans/coltrans_ros/data/takeoff_2cfs_payload_output.yaml", 50, 0)
                         ,("/home/whoenig/projects/tuberlin/col-trans/coltrans_ros/data/2cfs_payload_output_with_takeoff.yaml", 85, 5*85)]
    for motions_file_path, rate, repeat_last_setpoint  in motions_file_paths:
        with open(motions_file_path) as motions_file:
            motions = yaml.load(motions_file, Loader=yaml.FullLoader)

        dt = 1/rate
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
        quats = [state[3:7] for state in states]
        euler = rn.to_euler(rn.normalize(quats))
        # payload omega
        omegas = quat2omega(quats, dt)

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

        print("Executing trajectory" + str(traj_counter) + " ..." )
        executeTrajectory(timeHelper, allcfs, position, velocity, quats, omegas, cableAngleswithIDs, rate, offset=np.array([0, 0, 0]), repeat_last_setpoint=repeat_last_setpoint)
        traj_counter+=1
    if LOGGING:
        print("Logging done...")
        allcfs.setParam("usd.logging", 0)

    print("Landing...")
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()

    allcfs.setParam('stabilizer.controller', 6)
    
    allcfs.land(targetHeight=0.03, duration=3.0)
    timeHelper.sleep(4.0)

    # swarm = Crazyswarm()
    # timeHelper = swarm.timeHelper
    # allcfs = swarm.allcfs
 
    # allcfs.setParam('stabilizer.controller', 7)
    # allcfs.setParam('ctrlLeeP.form_ctrl', 3)
    # timeHelper.sleep(3.0) # extra time
    
    # allcfs.takeoff(targetHeight=HEIGHT, duration=3.0)
    # timeHelper.sleep(10.0) # extra time

    # rate = FREQUENCY
    # print("Executing trajectory...")
    # if LOGGING:
    #     print('Logging..')
    #     for cf in allcfs.crazyflies:
    #         cf.setParam("usd.logging", 1)
       
    # executeTrajectory(timeHelper, allcfs, position, velocity, quats, omegas, cableAngleswithIDs, FREQUENCY, offset=np.array([0, 0, HEIGHT]))

    # if LOGGING:
    #     print("Logging done...")
    #     for cf in allcfs.crazyflies:
    #         cf.setParam("usd.logging", 0)

    # print("Landing...")
    # for cf in allcfs.crazyflies:
    #     cf.notifySetpointsStop()

    # allcfs.setParam('stabilizer.controller', 6)
    
    # allcfs.land(targetHeight=0.03, duration=3.0)
    # timeHelper.sleep(4.0)

if __name__ == "__main__":
    main()
