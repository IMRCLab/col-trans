import numpy as np
from pathlib import Path
import yaml
import rowan as rn
from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory









def main():

    # parser = argparse.ArgumentParser()
    # parser.add_argument("motions", type=str, help="output file containing solution")
    # args = parser.parse_args()
    # motions_file_path = args.motions

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
 
    allcfs.setParam('stabilizer.controller', 7)
    allcfs.setParam('ctrlLeeP.form_ctrl', 3)
    # allcfs.emergency()

    if LOGGING:
        print('Logging..')
        allcfs.setParam("usd.logging", 1)
    timeHelper.sleep(3.0)

    traj_counter = 0
    # (filename, rate, repeat_last_setpoint)
    motions_file_paths = [("/home/whoenig/projects/tuberlin/col-trans/coltrans_ros/data/takeoff_2cfs_payload_output.yaml", 50, 0)
                         ,("/home/whoenig/projects/tuberlin/col-trans/coltrans_ros/data/2cfs_payload_output_with_takeoff.yaml", 85, 5*85)]
    for motions_file_path, rate, repeat_last_setpoint  in motions_file_paths:
        with open(motions_file_path) as motions_file:
            motions = yaml.load(motions_file, Loader=yaml.FullLoader)

    