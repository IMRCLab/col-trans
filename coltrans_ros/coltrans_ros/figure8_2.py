"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path

import numpy as np

TIMESCALE = 1.2
HEIGHT = 0.6
LOGGING = True

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # allcfs.emergency()

    traj1 = Trajectory()
    traj1.loadcsv("/home/whoenig/projects/crazyflie/crazyswarm2/src/coltrans_ros/data/figure8_3.csv")

    print("Upload trajectory")
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj1)

    # logging and takoff
    if LOGGING:
        allcfs.setParam('usd.logging', 1)
        timeHelper.sleep(2.0)

    allcfs.setParam('ring.effect', 7)
    allcfs.setParam('stabilizer.controller', 7)
    timeHelper.sleep(2.0)
    allcfs.takeoff(targetHeight=HEIGHT, duration=3.0)
    timeHelper.sleep(3.0)
    timeHelper.sleep(5.0) # extra time

    e = traj1.eval(0.0)

    # go to starting point
    for cf in allcfs.crazyflies:
        cf.goTo([0.25,0.0,HEIGHT],e.yaw,4.0)
        # cf.goTo([0.0,-0.25,HEIGHT],0,4.0)
    timeHelper.sleep(5.0)

    timeHelper.sleep(5.0) # extra time

    # start trajectory
    allcfs.startTrajectory(0, timescale=TIMESCALE, relative=False)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

    # go back to starting position and hover over floor
    for cf in allcfs.crazyflies:
        cf.goTo([0.5,-1.0,HEIGHT],0,4.0)
    timeHelper.sleep(5.0)

    # hover over floor
    for cf in allcfs.crazyflies:
        cf.goTo([0.5,-1.0,0.2],0,2.0)
    timeHelper.sleep(2.0)

    # Switch to another controller, so that the setpoint is not the payload
    allcfs.setParam('stabilizer.controller', 6)
    # timeHelper.sleep(1.0)

    # Land!
    allcfs.land(targetHeight=0.1, duration=1.5)
    timeHelper.sleep(1.5)

    allcfs.setParam('ring.effect', 0)
    if LOGGING:
        allcfs.setParam('usd.logging', 0)
        timeHelper.sleep(2.0)

if __name__ == "__main__":
    main()
