import sys
from importlib import reload
import controller
import yaml
import numpy as np
import random

class arguments():
    def __init__(self, filename, animate, plot, initUavs):
        self.filename = filename
        self.animate = animate
        self.plot = plot
        self.initUavs = initUavs

if __name__ == '__main__':
    sys.path.append("controllers/")

    with open('config/initialize.yaml') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)
    params["RobotswithPayload"]["payload"]["payloadCtrl"]["name"] = "lee"
    params["simtime"] = -1.022e3

    robotLoadSys = params["RobotswithPayload"]
    robots = robotLoadSys["Robots"]
    # for id, robot in robots.items():
    #     robot['q_dg'] = [random.randrange(-30, 30), random.randrange(-30, 30), random.randrange(0, 360)]
    #     robots[id]['q_dg'] = robot['q_dg']

    args = arguments("cftrial", False, False, False)
    animateOrPlotdict = {'animate':args.animate, 'plot':args.plot}
    print('\nsimulation using the python script\n')
    
    uavs_py, payload_py = controller.main(args, animateOrPlotdict, params)
    sys.path.remove("/home/khaledwahba94/imrc/col-trans/sim/controllers")
    sys.path.remove("controllers/")

    del sys.modules["controller"]
    del sys.modules["cffirmware"]

    sys.path.append("../crazyflie-firmware/")
    import controller
    params["RobotswithPayload"]["payload"]["payloadCtrl"]["name"] = "lee_firmware"
    print('\nsimulation using the firmware\n')
    
    uavs_fw, payload_fw = controller.main(args, animateOrPlotdict, params)


    mu_des_py = payload_py.mu_des_stack
    mu_des_fw = payload_fw.mu_des_stack
    # #extract data 
    qi_py = payload_py.plFullState[0:500,6:10]
    qi_fw = payload_fw.plFullState[0:500,6:10]
    er = qi_py.flatten() - qi_fw.flatten()
    # J = er.T@er
    # print(J,np.allclose(qi_py, qi_fw))

    # for uavpy, uavfw in zip(uavs_py.values(), uavs_fw.values()):
    #     for i, j in zip(range(len(uavpy.fullState)), range(len(uavfw.fullState))):
    #         print("statepy: ", uavpy.fullState[i,0:3])
    #         print("statefw: ", uavfw.fullState[j,0:3])


    # for i, j in zip(range(len(v_py)), range(len(v_fw))):
    #     print("1 v_py: ", v_py[i])
    #     print("1 v_fw: ", v_fw[j])


    # for i, j in zip(range(len(qi_py)), range(len(qi_fw))):
    #     print("1 qi_py: ", qi_py[i,0:3])
    #     print("1 qi_fw: ", qi_fw[j,0:3])
    #     print("2 qi_py: ", qi_py[i,3:6])
    #     print("2 qi_fw: ", qi_fw[j,3:6])
    #     print("3 qi_py: ", qi_py[i,6:9])
    #     print("3 qi_fw: ", qi_fw[j,6:9])
    # print()
    # for i, j in zip(range(len(wi_py)), range(len(wi_fw))):
    #     print("1 wi_py: ", wi_py[i,0:3])
    #     print("1 wi_fw: ", wi_fw[j,0:3])
    #     print("2 wi_py: ", wi_py[i,3:6])
    #     print("2 wi_fw: ", wi_fw[j,3:6])
    #     print("3 wi_py: ", wi_py[i,6:9])
    #     print("3 wi_fw: ", wi_fw[j,6:9])

    # for i, j in zip(range(len(mu_des_py)), range(len(mu_des_fw))):
    #     print("mu_py: ", mu_des_py[i,:])
    #     print("mu_fw: ", mu_des_fw[j,:])
    # for upar_py, upar_fw in zip(u_parpy, u_parfw):
    #     print('uparpy: ', upar_py)
    #     print('uparfw: ',upar_fw)
    # print()
    # for uper_py, uper_fw in zip(u_perpy, u_perfw):
    #     print('uperpy: ', uper_py)
    #     print('uperfw: ',uper_fw)
