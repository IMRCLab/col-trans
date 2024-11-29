#!/usr/bin/env python

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path

import numpy as np


TIMESCALE               = 1.0  # time scale of figure8 trajectory
TAKEOFF_DURATION        = 3.5 # Take off with payload (lee controller) 
PAYLOAD_CONTROLLER_TIME = 3.0  # Hover before starting traj (to stop swinging)
TARGET_HEIGHT           = 0.7  # Height of the payload

HOVER_BACK              = 5.0  

LOGGING = True
START_TRAJ = True
GOTO =  True
PAYLOAD_TAKEOFF_HEIGHT = 0.2
gotopos = [0.0, 0.0, PAYLOAD_TAKEOFF_HEIGHT]
goto_duration = 4.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # upload trajectory
    traj1 = Trajectory()
    traj1.loadcsv("/home/khaledwahba94/imrc/col-trans/coltrans_ros/data/figure8.csv")

    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj1)


    # set controller to lee + takeoff
    # allcfs.setParam('ring.effect', 7)
    allcfs.setParam('stabilizer.controller', 5)
    timeHelper.sleep(2.0)
    allcfs.takeoff(targetHeight=TARGET_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.5) # extra time

    # switch controller
    allcfs.setParam('stabilizer.controller', 7)
    print("switched controller.")
    timeHelper.sleep(PAYLOAD_CONTROLLER_TIME)
    
    # logging and takoff
    if LOGGING:
        allcfs.setParam('usd.logging', 1)
        timeHelper.sleep(2.0)

    # Hover with leePayload controller 
    # print('start hovering with lee payload')
    # cf.setParam('stabilizer.controller', 7)
    # timeHelper.sleep(PAYLOAD_CONTROLLER_TIME)
    
    ## Start infinity trajectory
    if START_TRAJ:
        print('start trajectory with lee payload')
        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        
        print('finished trajectory')
        allcfs.setParam("usd.logging", 0) 
    
    # elif GOTO:
    #     for cf in allcfs.crazyflies:
    #         cf.goTo(gotopos, 0.0, goto_duration)
    #     timeHelper.sleep(goto_duration+2.0) # extra time


    # Landing
    print("Landing...")
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()

    allcfs.setParam('stabilizer.controller', 5)
    allcfs.land(targetHeight=0.03, duration=3.0)
    timeHelper.sleep(4.0)

if __name__ == "__main__":
    main()
