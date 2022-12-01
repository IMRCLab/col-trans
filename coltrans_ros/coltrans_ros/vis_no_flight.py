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

    # get a fake setpoint
    allcfs.takeoff(1.0, 3.0)

    for cf in allcfs.crazyflies: 
        cf.setParam('stabilizer.controller', 7)


if __name__ == "__main__":
    main()
