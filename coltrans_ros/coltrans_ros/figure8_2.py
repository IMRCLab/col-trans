#!/usr/bin/env python

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path

import numpy as np


TARGET_HEIGHT     = 0.5  # Height
TAKEOFF_DURATION  = 3.0 # Take off with (lee controller) 
GOTO_DURATION     = TAKEOFF_DURATION - 2.5
TIMESCALE         = 0.9  # time scale of figure8 trajectory

LOGGING = True
START_TRAJ = True

CONTROLLER_TYPE = 5


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
    allcfs.setParam('stabilizer.controller', CONTROLLER_TYPE)
    timeHelper.sleep(2.0)
    allcfs.takeoff(targetHeight=TARGET_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.5) # extra time

    for cf in allcfs.crazyflies:
        cf.goTo(cf.initialPosition + np.array([0,0,TARGET_HEIGHT]), 0.0, GOTO_DURATION)
    timeHelper.sleep(GOTO_DURATION+1.0) # extra time


    # logging and takoff
    if LOGGING:
        allcfs.setParam('usd.logging', 1)
        timeHelper.sleep(2.0)

    ## Start infinity trajectory
    if START_TRAJ:
        print('start trajectory with lee payload')
        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
    
    print('finished trajectory')
    if LOGGING:
        allcfs.setParam("usd.logging", 0) 


    # Landing
    print("Landing...")
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop()

    allcfs.setParam('stabilizer.controller', CONTROLLER_TYPE)
    allcfs.land(targetHeight=0.03, duration=3.0)
    timeHelper.sleep(4.0)

if __name__ == "__main__":
    main()
