"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path

import numpy as np

TIMESCALE = 1.0
HEIGHT = 1.0
LOGGING = False

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # allcfs.emergency()

    traj1 = Trajectory()
    traj1.loadcsv("/home/whoenig/projects/crazyflie/crazyswarm2/src/coltrans_ros/data/figure8_2.csv")

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
    allcfs.takeoff(targetHeight=HEIGHT, duration=4.0)
    timeHelper.sleep(4.0)

    # go to origin
    for cf in allcfs.crazyflies:
        cf.goTo([0.0,0.0,HEIGHT],0,4.0)
    timeHelper.sleep(5.0)

    # start trajectory
    allcfs.startTrajectory(0, timescale=TIMESCALE, relative=False)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

    # go back to starting position
    for cf in allcfs.crazyflies:
        cf.goTo([0.5,-1.0,HEIGHT],0,4.0)
    timeHelper.sleep(5.0)

    # Switch to another controller, so that the setpoint is not the payload
    allcfs.setParam('stabilizer.controller', 6)
    timeHelper.sleep(1.0)

    # Land!
    allcfs.land(targetHeight=0.02, duration=3.5)
    timeHelper.sleep(3.5)

    allcfs.setParam('ring.effect', 0)
    if LOGGING:
        allcfs.setParam('usd.logging', 0)
        timeHelper.sleep(2.0)

if __name__ == "__main__":
    main()
