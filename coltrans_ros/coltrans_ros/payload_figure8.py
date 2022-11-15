#!/usr/bin/env python

import numpy as np
from pathlib import Path

from py_crazyswarm2 import *
from py_crazyswarm2.uav_trajectory import Trajectory

TIMESCALE          = 1.5  # time scale of figure8 trajectory
TAKEOFF_DURATION   = 10.0 # Take off with payload (lee controller) 
PAYLOAD_CONTROLLER = 8.0  # Hover before starting traj (to stop swinging)
TARGET_HEIGHT      = 1.0  # Height of the payload

HOVER_BACK         = 5.0  

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / "data/figure8.csv")

## Upload figure8 Trajectory
    cf.uploadTrajectory(0, 0, traj1)

## Take off with Lee controller
    cf.setParam('stabilizer.controller', 6)    
    cf.takeoff(targetHeight=TARGET_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)    
    
## Hover with lee UAV-payload controller 
    print('start hovering with lee payload')
    cf.setParam('stabilizer.controller', 7)
    timeHelper.sleep(PAYLOAD_CONTROLLER)


    cf.setParam("usd/logging", 1)

## Start infinity trajectory
    cf.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
    
    print('finished trajectory')
    cf.setParam("usd.logging", 0) 

    print('swap controller')
    cf.setParam('ctrlLee.mass', 0.034+0.007)
    cf.setParam('stabilizer.controller', 6)

    cf.land(targetHeight=0.04, duration=2.0)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
