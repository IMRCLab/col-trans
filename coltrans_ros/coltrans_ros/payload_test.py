"""This script is for testing the payload controller with two UAVs"""

# from pycrazyswarm import Crazyswarm
from py_crazyswarm2 import Crazyswarm
import numpy as np




Ids = [2, 6]
TAKEOFF_HEIGHT = 1.0
Heights = [1.0, 1.0]

   
cf_config = {
    2: {
        'waypoints': [
            [0.5, -0.5, 1.5],
            [0.0, -0.5, 1.0],
        ]
    },
    6: {
        'waypoints': [
            [0.5, 0.5, 0.5],
            [0.0, 0.5, 1.0],
        ]
    }
}


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=3.0)
    timeHelper.sleep(3.5)
    
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, TAKEOFF_HEIGHT])
        cf.goTo(pos, 0, 3.0)

    timeHelper.sleep(5)

    for i in range(2):
        for cfid in Ids:
            pos = np.array(cf_config[cfid]['waypoints'][i])
            print(cfid, pos)
            allcfs.crazyfliesById[cfid].goTo(pos, 0, 5.0)
        timeHelper.sleep(5.5) 
    
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, TAKEOFF_HEIGHT])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(5)


    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(5.0)