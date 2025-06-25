#!/usr/bin/env python

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path

import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = Trajectory()
    # traj1.loadcsv("/home/kwahba/col-trans/coltrans_ros/data/excitatory2_traj.csv")
    traj1.loadcsv("/home/kwahba/col-trans/coltrans_ros/data/yaw0.csv")
    # traj1.loadcsv("/home/kwahba/col-trans/coltrans_ros/data/figure8.csv")

    TRIALS = 4
    TIMESCALE = 1.6
    for i in range(TRIALS):
        for cf in allcfs.crazyflies:
            cf.uploadTrajectory(0, 0, traj1)

        allcfs.setParam('stabilizer.controller', 2)

        allcfs.takeoff(targetHeight=0.6, duration=3.0)
        timeHelper.sleep(4.)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 0.6])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(3.0)

        for cf in allcfs.crazyflies:
            cf.setParam("usd.logging", 1)

        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        for cf in allcfs.crazyflies:
            cf.setParam("usd.logging", 0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()
