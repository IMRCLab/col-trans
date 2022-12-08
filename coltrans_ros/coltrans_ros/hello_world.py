"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path

import numpy as np

TIMESCALE = 2.0
UAVSHEIGHT = 0.8
HEIGHT = 0.4


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # allcfs.emergency()

    traj1 = Trajectory()
    traj1.loadcsv("/home/khaledwahba94/imrc/col-trans/coltrans_ros/data/figure8.csv")

    print("Upload trajectory")
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj1)

    #### set controller to lee for take off
    # print('take off with lee controller')

    for cf in allcfs.crazyflies:
        cf.setParam('stabilizer.controller', 6)
    ## take off 
    allcfs.takeoff(targetHeight=UAVSHEIGHT, duration=2.0)
    timeHelper.sleep(10.0)
    ####

    ## start the QP lee Payload controller for hovering

    print('start hovering with QP lee payload')
    for cf in allcfs.crazyflies:
        cf.setParam('stabilizer.controller', 7)
    
    timeHelper.sleep(5.0)

    # allcfs.takeoff(targetHeight=HEIGHT, duration=5.0)
    # timeHelper.sleep(6.0)

    # go to origin
    # for cf in allcfs.crazyflies:
    #     cf.goTo([0.0,0.0,HEIGHT],0,4.0)
    # timeHelper.sleep(5.0)

    # # start trajectory
    # allcfs.startTrajectory(0, timescale=TIMESCALE)
    # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

    # Land the payload
    for cf in allcfs.crazyflies:
        cf.goTo([0.0,0.0,-0.1],0,4.0)
    timeHelper.sleep(5.0)

    # Switch to another controller, so that the setpoint is not the payload
    for cf in allcfs.crazyflies:
        cf.setParam('stabilizer.controller', 6)
    timeHelper.sleep(2.0)

    # Land!
    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()
