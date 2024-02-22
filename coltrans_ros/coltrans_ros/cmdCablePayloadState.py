import numpy as np
from pathlib import Path
import yaml
import rowan as rn
from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory


np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

def derivative(vec, dt):
    dvec  =[[0,0,0]]
    for i in range(len(vec)-1):
        dvectmp = (vec[i+1]-vec[i])/dt
        dvec.append(dvectmp)
    return np.asarray(dvec)

def executeTrajectory(timeHelper, allcfs, position, velocity, acceleration, cablesPlannedwithIDs, rate=100, repeat_last_setpoint=0, quats=np.array([0,0,0,np.nan]), pm=True):
    
    start_time = timeHelper.time()
    i = 0
    
    while not timeHelper.isShutdown():

        if i >= len(velocity) + repeat_last_setpoint:
            break
        idx = min(i, len(velocity)-1)
        # print("idx: ",idx)
        # print(cablesPlannedwithIDs[idx])
        pos = position[idx]
        vel = velocity[idx]
        # acc = acceleration[idx]
        acc = np.zeros(3,)
        if not pm:
            quat = quats[idx]
            omega = omegas[idx]
        else: 
            quat = quats.copy()
        allcfs.cmdFullState(
            pos,
            vel,
            acc,
            quat,
            np.zeros_like(vel))
        # print(cablesPlannedwithIDs[idx])
        allcfs.cmdDesCableStates(cablesPlannedwithIDs[idx])
        i+=1
        timeHelper.sleepForRate(rate)
    print("Trajectory executed")





def main():

    IDs = [2, 3, 7] # TODO: this shouldn't be like this
    LOGGING = True
    EMERGENCY = False
    TAKEOFF = True
    TRAJ = True
    TAKEOFF_HEIGHT = 1.0
    PAYLOAD_TAKEOFF_HEIGHT = 0.5
    TAKEOFF_DURATION = 3.0
    RATE = 70
    num_robots = len(IDs)    

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # timeHelper.sleep(10.0) # extra time
    if EMERGENCY:
        allcfs.emergency()
        print("In emergency mode")

    # timeHelper.sleep(10.0)
    if TAKEOFF: 
        print("set controller to lee + takeoff")
        allcfs.setParam('ring.effect', 7)
        allcfs.setParam('stabilizer.controller', 6)
        timeHelper.sleep(1.0)
        allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION + 1.) # extra time
     

    print("Set controller to LeePayload.")
    allcfs.setParam('stabilizer.controller', 7)
    print("Params are set.")
    if TAKEOFF:
        timeHelper.sleep(1.0) # extra time
        # go to starting point
        for cf in allcfs.crazyflies:
            cf.goTo([0.0,-1.0, PAYLOAD_TAKEOFF_HEIGHT],0.0, 3.0)
        timeHelper.sleep(3.5) # extra time

    print("#########")
    print("Loading file...")
    traj_counter = 0
    # (filename, rate, repeat_last_setpoint)
    motions_file_paths = [
        # ("/home/khaledwahba94/imrc/ros2_ws/src/coltrans_ros/data/2cfs_takeoff/opt/trajectory.yaml", RATE, 0),
        # ("/home/khaledwahba94/imrc/ros2_ws/src/coltrans_ros/data/2cfs_window/geom/trajectory.yaml", RATE, 0),
        ("/home/khaledwahba94/imrc/ros2_ws/src/coltrans_ros/data/3cfs_forest/opt/trajectory.yaml", RATE, 0),
                            ]
    for motions_file_path, rate, repeat_last_setpoint  in motions_file_paths:
        with open(motions_file_path) as motions_file:
            motions = yaml.load(motions_file, Loader=yaml.FullLoader)
        print("File loaded!")
        print("#########")
        # state = {"xp [m]",     "yp [m]",      "zp [m]",      "vpx [m/s]",
                # "vpy [m/s]",  "vpz [m/s]",   "qcx []",      "qcy []",
                # "qcz[]",      "wcx [rad/s]", "wcy [rad/s]", "wcz [rad/s]",
                # "qx []",      "qy []",       "qz []",       "qw []",
                # "wx [rad/s]", "wy [rad/s]",  "wz [rad/s]"}
        states_d = motions["result"]["states"]
        mu_planned = motions["result"]["mu_planned"]

        position = np.asarray([state[0 : 3] for state in states_d])
        if TAKEOFF:
            for i, pos in enumerate(position):
                pos[2] += PAYLOAD_TAKEOFF_HEIGHT
                position[i] = pos
        velocity = np.asarray([state[3 : 6] for state in states_d])
        acceleration = derivative(velocity, 0.01)
        cables   = np.asarray([state[6 : 6+6*num_robots] for state in states_d])        
        cablesPlannedwithIDs = [] #[id, mu_planned, qidot_planned]
        for cable, mu_planned_i in zip(cables, mu_planned): 
            data_tmp = []
            for i in range(num_robots):
                mu_i = mu_planned_i[3*i : 3*i+3]
                q_i  = cable[6*i : 6*i +3]
                w_i  = cable[6*i + 3 : 6*i + 6]
                qdot_i = np.cross(w_i, q_i).tolist()
                data_tmp.append((IDs[i], mu_i, qdot_i))
            cablesPlannedwithIDs.append(data_tmp)
        
        timeHelper.sleep(1.0)
        # timeHelper.sleep(3.0)
        if LOGGING:
            print("########")
            print('Logging..')
            allcfs.setParam("usd.logging", 1)
            timeHelper.sleep(2.0)

        print("########")
        if TRAJ: 
            print("Activate formation control...")
            allcfs.setParam('ctrlLeeP.form_ctrl', 3)
            print("########")
            print("Ready to execute trajectory...")
            print("Executing trajectory" + str(traj_counter) + " ..." )
            executeTrajectory(timeHelper, allcfs, position, velocity, acceleration, cablesPlannedwithIDs, rate, repeat_last_setpoint=repeat_last_setpoint)
            traj_counter+=1


    if LOGGING:
        print("Logging done...")
        allcfs.setParam("usd.logging", 0)
        timeHelper.sleep(1.0)
    
    print("Landing...")
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()

    allcfs.setParam('stabilizer.controller', 6)
    
    allcfs.land(targetHeight=0.03, duration=3.0)
    timeHelper.sleep(4.0)


if __name__ == "__main__":
    main()
