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
        acc = acceleration[idx]
        # acc = np.zeros(3,)
        if not pm:
            quat = quats[idx]
        else: 
            quat = quats.copy()
        allcfs.cmdFullState(
            pos,
            vel,
            acc,
            quat,
            np.zeros(vel.shape))
        # print(cablesPlannedwithIDs[idx])
        allcfs.cmdDesCableStates(cablesPlannedwithIDs[idx])
        i+=1
        timeHelper.sleepForRate(rate)
    print("Trajectory executed")


def main():

    IDs = [3,7,9] # TODO: this shouldn't be like this
    # IDs = [3,9] # TODO: this shouldn't be like this
    LOGGING = True
    EMERGENCY = False
    TAKEOFF = True
    TRAJ = True
    TAKEOFF_HEIGHT = 0.9
    PAYLOAD_TAKEOFF_HEIGHT = 0.4
    TAKEOFF_DURATION = 4.0
    RATE = 100
    CONTROLLER_TYPE = 2

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
        allcfs.setParam('ring.effect', 0)
        allcfs.setParam('stabilizer.controller',CONTROLLER_TYPE)
        timeHelper.sleep(1.0)
        allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION + 1.) # extra time
     

    print("Set controller to LeePayload.")
    allcfs.setParam('stabilizer.controller', 7)
    print("Params are set.")

    print("Go to payload initial position")
    # allcfs.goTo(np.array([0.0, -1.0, PAYLOAD_TAKEOFF_HEIGHT]), 0, 1.0, relative=False) # forrest
    allcfs.goTo(np.array([-1.0, 0.0, PAYLOAD_TAKEOFF_HEIGHT]), 0, 1.0, relative=False) # window

    timeHelper.sleep(1.2) # extra time

    print("#########")
    print("Loading file...")
    traj_counter = 0
    # (filename, rate, repeat_last_setpoint)
    motions_file_paths = [
        # window 2 robots coltrans
        # ("/home/kwahba/ros2_ws/src/coltrans_ros/data/coltrans_flights/2cfs/window/window_2robots_coltrans.yaml", RATE, 0),
        
        # window 2 robots dbcbs
        # ("/home/kwahba/ros2_ws/src/coltrans_ros/data/dbcbs_flights/2cfs/window/window_2robots_dbcbs.yaml", RATE, 0),
        
        # forest 2 robots coltrans
        # ("/home/kwahba/ros2_ws/src/coltrans_ros/data/coltrans_flights/2cfs/forest/forest_2robots_coltrans.yaml", RATE, 0),
       
        # forest 2 robots dbcbs
        # ("/home/kwahba/ros2_ws/src/coltrans_ros/data/dbcbs_flights/2cfs/forest/forest_2robots_dbcbs.yaml", RATE, 0),
        
        # window 3 robots coltrans
        # ("/home/kwahba/ros2_ws/src/coltrans_ros/data/coltrans_flights/3cfs/window/window_3robots_coltrans.yaml", RATE, 0),
        
        # window 3 robots dbcbs
        ("/home/kwahba/ros2_ws/src/coltrans_ros/data/dbcbs_flights/3cfs/window/window_3robots_dbcbs.yaml", RATE, 0),
        
        # forest 3 robots coltrans
        # ("/home/kwahba/ros2_ws/src/coltrans_ros/data/coltrans_flights/3cfs/forest/forest_3robots_coltrans.yaml", RATE, 0),

        # forest 3 robots dbcbs
        # ("/home/kwahba/ros2_ws/src/coltrans_ros/data/dbcbs_flights/3cfs/forest/forest_3robots_dbcbs.yaml", RATE, 0),


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
        states_d = motions["result"]["refstates"]
        mu_planned = motions["result"]["mu_planned"]

        position = np.asarray([state[0 : 3] for state in states_d])
        for i, pos in enumerate(position):
            pos[2] += PAYLOAD_TAKEOFF_HEIGHT
            position[i] = pos
        velocity = np.asarray([state[3 : 6] for state in states_d])
        acceleration = np.asarray([state[6 : 9] for state in states_d])
        cables   = np.asarray([state[9 : 9+6*num_robots] for state in states_d])        
        cablesPlannedwithIDs = [] #[id, mu_planned, qidot_planned]
        for cable, mu_planned_i in zip(cables, mu_planned): 
            data_tmp = []
            for i in range(num_robots):
                mu_i = mu_planned_i[3*i : 3*i+3]                
                mu_i[2] = np.clip(mu_i[2], 0., 1)
                q_i  = cable[6*i : 6*i +3]
                w_i  = cable[6*i + 3 : 6*i + 6]
                qdot_i = np.cross(w_i, q_i).tolist()
                data_tmp.append((IDs[i], mu_i, qdot_i))
            cablesPlannedwithIDs.append(data_tmp)
        
        timeHelper.sleep(0.5)
        if LOGGING:
            print('Logging..')
            allcfs.setParam("usd.logging", 1)
            timeHelper.sleep(1.0)

        print("########")
        if TRAJ: 
            print("Activate formation control and cable tracking...")
            allcfs.setParam('ctrlLeeP.en_qdidot', 1)
            allcfs.setParam('ctrlLeeP.form_ctrl', 3)
            # timeHelper.sleep(1.0)
            print("########")
            print("Ready to execute trajectory...")
            print("Executing trajectory" + str(traj_counter) + " ..." )
            executeTrajectory(timeHelper, allcfs, position, velocity, acceleration, cablesPlannedwithIDs, rate, repeat_last_setpoint=repeat_last_setpoint)
            traj_counter+=1


    # Landing
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()


    if LOGGING:
        print("Logging done...")
        allcfs.setParam("usd.logging", 0)


    print("Landing...")
    allcfs.setParam('stabilizer.controller', CONTROLLER_TYPE)

    allcfs.land(targetHeight=0.03, duration=3.0)




if __name__ == "__main__":
    main()
