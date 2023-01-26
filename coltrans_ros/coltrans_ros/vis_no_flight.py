"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
from crazyflie_py import Crazyswarm
import numpy as np

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # emergency to prevent flight
    allcfs.emergency()

    for cf in allcfs.crazyflies: 
        cf.setParam('ctrlLeeP.lambda', 0.0)
        cf.setParam('stabilizer.controller', 7)

    allcfs.land(0.0, 0.5)
    timeHelper.sleep(1.0)


    # get a fake setpoint
    allcfs.takeoff(-0.5, 3.0)

    # timeHelper.sleep(3.0)

    # for cf in allcfs.crazyflies: 
    #     cf.setParam('usd.logging', 1)

    # timeHelper.sleep(30.0)

    # for cf in allcfs.crazyflies: 
    #     cf.setParam('usd.logging', 0)


if __name__ == "__main__":
    main()
