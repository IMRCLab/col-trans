"""Test the updated firmware and the new motion planner"""

# from pycrazyswarm import Crazyswarm
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path

import numpy as np

TIMESCALE = 1.2
HEIGHT = 0.5
LOGGING = False


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # allcfs.emergency()
    traj1 = Trajectory()
    traj1.loadcsv("/home/khaledwahba94/imrc/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/figure8.csv")

    print("Upload trajectory")
    # for cf in allcfs.crazyflies:
    #     cf.uploadTrajectory(0, 0, traj1)
    # logging and takoff
    if LOGGING:
        allcfs.setParam('usd.logging', 1)
        timeHelper.sleep(2.0)

    allcfs.setParam('ring.effect', 7)
    allcfs.setParam('stabilizer.controller', 7)
    print("Controller activated and taking off...")
    timeHelper.sleep(3.0)
    allcfs.takeoff(targetHeight=HEIGHT, duration=3.0)
    timeHelper.sleep(3.5)


    # go to starting point
    for cf in allcfs.crazyflies:
        cf.goTo([0.0,0.0,HEIGHT],0,2.0)
        # cf.goTo([0.0,-0.25,HEIGHT],0,4.0)
    timeHelper.sleep(3.0)

    # timeHelper.sleep(5.0) # extra time

    # # start trajectory
    # print("Start trajectory...")
    # timeHelper.sleep(2.0)
    # allcfs.startTrajectory(0, timescale=TIMESCALE, relative=False)
    # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
    # print("Trajectory executed...")

    # # go back to starting position and hover over floor
    # for cf in allcfs.crazyflies:
    #     cf.goTo([0.0,0.0,HEIGHT],0,1.0)
    # timeHelper.sleep(2.0)

    # hover over floor
    for cf in allcfs.crazyflies:
        cf.goTo([0.0,0.0,0.2],0,2.0)
    timeHelper.sleep(2.0)

    # Switch to another controller, so that the setpoint is not the payload
    print("Switching controller and landing...")
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
