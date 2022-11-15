import numpy as np
from pathlib import Path

from py_crazyswarm2 import *
from py_crazyswarm2.uav_trajectory import Trajectory

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / "data/yaw0.csv")
    ev = traj1.eval(0)
    
    TRIALS = 1
    TIMESCALE = 5.0
    for i in range(TRIALS):
        for cf in allcfs.crazyflies:
            cf.uploadTrajectory(0, 0, traj1)

        allcfs.takeoff(targetHeight=0.5, duration=3.0)
        timeHelper.sleep(5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
            cf.goTo(pos, ev.yaw, 3.0)
        timeHelper.sleep(4.0)
        
        for cf in allcfs.crazyflies:
            cf.setParam("usd/logging", 1)

        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        for cf in allcfs.crazyflies:
            cf.setParam("usd/logging", 0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()
