import numpy as np
import rowan as rn
import matplotlib.pyplot as plt
from uavDy import uav
from uavDy.uav import skew
from Animator import animateSingleUav 
from trajectoriescsv import *
import time
import argparse
import sys
from itertools import permutations, combinations, chain
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)
import cffirmware
import yaml

class hyperplane:
    def __str__(self):
      return  "normal: [{}, {}, {}], offset: {} ".format(np.around(self.n[0], decimals=5), np.around(self.n[1], decimals=5), np.around(self.n[2], decimals=5), np.around(self.a, decimals=5))
    def __init__(self, n, a):
        self.n = n
        self.a = a
    def coeffs(self):
        return np.array([self.n[0], self.n[1], self.n[2], self.a])

def initController(controller):
    """This function initializes the controller"""
    if controller['name'] == 'lee':
        cffirmware.controllerLeeInit()
    elif controller['name'] == 'lee_firmware':
        lee = cffirmware.controllerLee_t()
        cffirmware.controllerLeeInit(lee)
        lee.Kpos_P.x = controller['kpx']
        lee.Kpos_P.y = controller['kpy']
        lee.Kpos_P.z = controller['kpz']
        lee.Kpos_D.x = controller['kdx']
        lee.Kpos_D.y = controller['kdy']
        lee.Kpos_D.z = controller['kdz']
        lee.Kpos_I.x = controller['kipx']
        lee.Kpos_I.y = controller['kipy']
        lee.Kpos_I.z = controller['kipz']
        lee.KR.x     = controller['krx']
        lee.KR.y     = controller['kry']
        lee.KR.z     = controller['krz']
        lee.Komega.x = controller['kwx']
        lee.Komega.y = controller['kwy']
        lee.Komega.z = controller['kwz']
        lee.KI.x     = controller['kix']
        lee.KI.y     = controller['kiy']
        lee.KI.z     = controller['kiz']   
    
        control = cffirmware.control_t()
        # allocate desired state
        setpoint = cffirmware.setpoint_t()
        setpoint = setTrajmode(setpoint)
        sensors = cffirmware.sensorData_t()
        state = cffirmware.state_t()
        
        return lee, control, setpoint, sensors, state
    elif controller['name'] == 'sjc_firmware':
        cffirmware.controllerSJCInit()
    # Allocate output variable
    # For this example, only thrustSI, and torque members are relevant
    control = cffirmware.control_t()
    # allocate desired state
    setpoint = cffirmware.setpoint_t()
    setpoint = setTrajmode(setpoint)
    sensors = cffirmware.sensorData_t()
    state = cffirmware.state_t()
    
    return control, setpoint, sensors, state 

def setTrajmode(setpoint):
    """This function sets the trajectory modes of the controller"""
    setpoint.mode.x = cffirmware.modeAbs
    setpoint.mode.y = cffirmware.modeAbs
    setpoint.mode.z = cffirmware.modeAbs
    setpoint.mode.quat = cffirmware.modeAbs
    setpoint.mode.roll = cffirmware.modeDisable
    setpoint.mode.pitch = cffirmware.modeDisable
    setpoint.mode.yaw = cffirmware.modeDisable
    return setpoint

def updateDesState(setpoint, controller, fulltraj):
    """This function updates the desired states"""
    setpoint.position.x = fulltraj[0]  # m
    setpoint.position.y = fulltraj[1]  # m
    setpoint.position.z = fulltraj[2]  # m
    setpoint.velocity.x = fulltraj[3]  # m/s
    setpoint.velocity.y = fulltraj[4]  # m/s
    setpoint.velocity.z = fulltraj[5]  # m/s
    setpoint.acceleration.x = fulltraj[6]  # m/s^2
    setpoint.acceleration.y = fulltraj[7]  # m/s^2
    setpoint.acceleration.z = fulltraj[8]  # m/s^2
    setpoint.attitude.yaw = 0  # deg
    if len(fulltraj) == 15 and (controller['name'] == 'lee' \
    or controller['name'] == 'lee_firmware'):
        setpoint.jerk.x = fulltraj[9]
        setpoint.jerk.y = fulltraj[10]
        setpoint.jerk.z = fulltraj[11]
        if controller['name'] == 'lee':
            setpoint.snap.x = fulltraj[12]
            setpoint.snap.y = fulltraj[13]
            setpoint.snap.z = fulltraj[14]
    elif len(fulltraj) == 9 and (controller['name'] == 'lee' \
    or controller['name'] == 'lee_firmware'):
        setpoint.jerk.x = 0 
        setpoint.jerk.y = 0 
        setpoint.jerk.z = 0 
        if controller['name'] == 'lee':
            setpoint.snap.x = 0 
            setpoint.snap.y = 0 
            setpoint.snap.z = 0 
    return setpoint
    
def updateSensor(sensors, uav):
    """This function updates the sensors signals"""
    uavState = uav.state
    sensors.gyro.x = np.degrees(uavState[10]) # deg/s
    sensors.gyro.y = np.degrees(uavState[11]) # deg/s
    sensors.gyro.z = np.degrees(uavState[12]) # deg/s
    return sensors

def updateState(state, uav):
    """This function passes the current states to the controller"""
    uavState = uav.state
    state.position.x = uavState[0]   # m
    state.position.y = uavState[1]    # m
    state.position.z = uavState[2]    # m
    state.velocity.x = uavState[3]    # m/s
    state.velocity.y = uavState[4]    # m/s
    state.velocity.z = uavState[5]    # m/s
    q_curr = np.array(uavState[6:10]).reshape((4,))
    rpy_state  = rn.to_euler(q_curr,convention='xyz')
    state.attitude.roll  = np.degrees(rpy_state[0])
    state.attitude.pitch = np.degrees(-rpy_state[1])
    state.attitude.yaw   = np.degrees(rpy_state[2])
    state.attitudeQuaternion.w = q_curr[0]
    state.attitudeQuaternion.x = q_curr[1]
    state.attitudeQuaternion.y = q_curr[2]
    state.attitudeQuaternion.z = q_curr[3]
    fullState = np.array([state.position.x,state.position.y,state.position.z, 
                          state.velocity.x,state.velocity.y, state.velocity.z, 
                          q_curr[0],q_curr[1],q_curr[2],q_curr[3], uavState[10],uavState[11],uavState[12]]).reshape((13,))
    
    return state, fullState

def initializeState(uav_params):
    """This function sets the initial states of the UAV
        dt: time step
        initPose: initial position [x,y,z]
        initq: [qw, qx, qy, qz] initial rotations represented in quaternions 
        initLinVel: [xdot, ydot, zdot] initial linear velocities
        initAngVel: [wx, wy, wz] initial angular velocities"""
    dt = float(uav_params['dt'])
    
    initPos = np.array(uav_params['init_pos_Q'])
    
    # initialize Rotation matrix about Roll-Pitch-Yaw
    attitude = uav_params['init_attitude_Q'] 
    for i in range(0,len(attitude)):
        attitude[i] = np.radians(attitude[i])
    initq = rn.from_euler(attitude[0],attitude[1],attitude[2])    
    
    #Initialize Twist
    initLinVel = np.array(uav_params['init_linVel_Q'])
    initAngVel = np.array(uav_params['init_angVel_Q'])
    ### State = [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, wx, wy, wz] ###
    initState = np.zeros((13,))
    initState[0:3]  = initPos  # position: x,y,z
    initState[3:6]  = initLinVel  # linear velocity: xdot, ydot, zdot
    initState[6:10] = initq# quaternions: [qw, qx, qy, qz]
    initState[10::] = initAngVel # angular velocity: wx, wy, wz
    return dt, initState

def initializeStateWithPayload(payload_cond):
    """This function sets the initial states of the UAV-Payload system
        dt: time step
        initPose: initial payload position [xl,yl,zl]
        initLinVel: [xldot, yldot, zldot] initial linear velocities
        initp: initial directional unit vector pointing from UAV to payload expressed in Inertial frame
        initq: [qw, qx, qy, qz] initial rotations represented in quaternions 
        initAngVel: [wx, wy, wz] initial angular velocities"""

    dt = float(payload_cond['dt'])
    lc = float(payload_cond['l_c']) # length of cable [m] 
    
    initPosL = np.array(payload_cond['init_pos_L']) #  Initial position
    initp    = np.array(payload_cond['p']) #  Initial Unit vector

     #Initialize payload Twist
    inLinVL  = np.array(payload_cond['init_linV_L']) # Linear velocity of payload
    inAnVL   = np.array(payload_cond['wl']) # Angular Velocity of Payload
    
    # initialize Rotation matrix: Roll-Pitch-Yaw
    attitude = payload_cond['init_attitude_Q'] 
    for i in range(0,len(attitude)):
        attitude[i] = np.radians(attitude[i])
    initq = rn.from_euler(attitude[0],attitude[1],attitude[2])    

    # Initialize anglular velocity of quadrotor
    initAngVel = np.array(payload_cond['init_angVel_Q'])
    initState  = np.zeros((19,))

    initState[0:3]   = initPosL
    initState[3:6]   = inLinVL
    initState[6:9]   = initp
    initState[9:12]  = inAnVL
    initState[12:16] = initq
    initState[16::]  = initAngVel
    return dt, initState

def StQuadfromPL(payload):
    """This function initializes the states of the quadrotor given the states of the payload """ 
    uavState =  np.zeros((13,))
    posq = payload.state[0:3] - payload.lc * payload.state[6:9]
    pdot = np.cross(payload.state[9:12], payload.state[6:9])
    velq = payload.state[3:6] - payload.lc * pdot
    uavState[0:3]  = posq
    uavState[3:6]  = velq
    uavState[6:10] = payload.state[12:16]
    uavState[10::] =  payload.state[16::]
    return uavState

def animateTrajectory(uavs, payloads, videoname, shared, sample):
    # Animation    
    fig     = plt.figure(figsize=(10,10))
    ax      = fig.add_subplot(autoscale_on=True,projection="3d")
    animate = animateSingleUav.PlotandAnimate(fig, ax, uavs, payloads, sample, shared) 
    dt_sampled = list(uavs.values())[0].dt * sample
    print("Starting Animation... \nAnimating, Please wait...")
    now = time.time()
    startanimation = animate.startAnimation(videoname,dt_sampled)
    print("Converting Animation to Video. \nPlease wait...")

    end = time.time()
    plt.close(fig)
    print("Run time:  {:.3f}s".format((end - now)))

def animateOrPlot(uavs, payloads, animateOrPlotdict, filename, tf_sim, shared, sample): 
    # The plot will be shown eitherways
    # savePlot: saves plot in pdf format
    if animateOrPlotdict['plot']:
        pdfName = filename + '.pdf'
        animateSingleUav.outputPlots(uavs, payloads, tf_sim, pdfName, shared)

    if animateOrPlotdict['animate']:
        videoname = filename + '.gif'
        animateTrajectory(uavs, payloads, videoname, shared, sample)     
  
def setParams(params):
    dt           = float(params['dt'])
    uavs, payloads, trajectories  = {}, {}, {}
    
    for name, robot in params['Robots'].items():
        trajectories['uav_'+name]   = robot['refTrajPath']
        if robot['payload']['mode'] == 'enabled':
            payload_params          = {**robot['payload'], **robot['initConditions'], 'm':robot['m'], 'dt':dt}
            dt, initState           = initializeStateWithPayload(payload_params)
            payload                 = uav.Payload(dt, initState, payload_params)
            uav1                    = uav.UavModel(dt, 'uav_'+name, StQuadfromPL(payload), robot, pload=True, lc=payload.lc)
            uavs['uav_'+name]       = uav1
            payloads['uav_'+name] = payload
        else:
            uav_params     = {'dt': dt, **robot['initConditions'], **robot}
            dt, initState  = initializeState(uav_params)
            uav1           = uav.UavModel(dt, 'uav_'+name, initState, uav_params) 
            uavs['uav_'+name] = uav1
    return uavs, payloads, trajectories        

def StatefromSharedPayload(id, payload, angState, lc, j):
    ## Thid method computes the initial conditions of each quadrotor
    #  given the initial condition of the payload and the directional unit vectors of each cable
    qi = payload.state[j:j+3]
    wi = payload.state[j+3*payload.numOfquads:j+3+3*payload.numOfquads]
    uavState =  np.zeros((13,))
    posq =  payload.state[0:3] - lc * qi
    pdot = np.cross(wi, qi)
    velq = payload.state[3:6] - lc * pdot
    if not payload.pointmass:
        R0   = rn.to_matrix(payload.state[6:10])
        posFrload = payload.posFrloaddict[id]
        posq = payload.state[0:3] - lc * qi + R0@posFrload
        wl = payload.state[10:13] #wl of payload
        R0_dot = R0@uav.skew(wl)
        velq = payload.state[3:6] - lc * pdot + R0_dot @ posFrload
    uavState[0:3]  = posq
    uavState[3:6]  = velq
    uavState[6:10] = angState[0:4]
    uavState[10::] =  angState[4:]
    return uavState

def setPayloadfromUAVs(uavs_params, payload_params):
    ## THIS IS NOT USED NOW!!
    ## This method sets the states of the payload given the positions of the quadrotor. 
    ## This is opposite to what is normally done, but since the controller for the whole system
    ## has not been yet finished, then provided an initial condition of all UAVs and the length of the cables,
    ## the payload initial conditions are computed. it is activated through --initUavs flag argument
    for params in uavs_params.values():
        posq = np.array(params['init_pos_Q'])
        lc   = params['l_c']
        vq   = np.array(params['init_linVel_Q'])
        angR = np.radians(params['q_dg'])
        q    =  rn.to_matrix(rn.from_euler(angR[0], angR[1], angR[2], convention='xyz',axis_type='extrinsic')) @ np.array([0,0,-1]) #
        qdot = np.array(params['qd'])
        initPos  = posq + lc * q
        initLinV = vq + lc * qdot
    payload_params.update({'init_pos_L': initPos, 'init_linV_L': initLinV})
    return payload_params, uav.SharedPayload(payload_params, uavs_params)
    pass

def setTeamParams(params, initUavs):
    dt    = float(params['dt'])
    uavs, trajectories, pltrajectory = {}, {}, {}
    plStSize = 13 # 13 is the number of the payload states.
            #  We want to get the angles and its derivatives
            #  between load and UAVs (Check the state structure of SharedPayload object)
    inertia = np.diag(np.array(params['RobotswithPayload']['payload']['inertia']))
    if np.linalg.det(inertia) == 0:
            plStSize -= 7 # if the payload is considered as a point mass than we only have the linear terms 
                          # thus the state: [xp, yp, zp, xpdot, ypdot, zpdot]
    ## --initUavs: this flag let us initialize the conditions of the payload, given the initial condtions
    ## of the UAVs (which is not what is normally done, but for the sake of having easier tests).
    if not initUavs:
        for key in (params['RobotswithPayload']['payload']).keys():
            if key == 'refTrajPath':
                pltrajectory = params['RobotswithPayload']['payload']['refTrajPath']
        payload_params = {**params['RobotswithPayload']['payload'], 'dt': dt}
        uavs_params = {}
        for name, robot in params['RobotswithPayload']['Robots'].items():
            trajectories['uav_'+name]   = robot['refTrajPath']
            uavs_params.update({name: {**robot}})
        payload = uav.SharedPayload(payload_params, uavs_params)
        j = plStSize
        for name, robot in uavs_params.items():
            lc     = robot['l_c']
            eulAng = robot['initConditions']['init_attitude_Q']
            quat   = rn.from_euler(eulAng[0], eulAng[1], eulAng[2])
            w_i    = robot['initConditions']['init_angVel_Q']
            angSt  = np.hstack((quat, w_i)).reshape((7,))
            uav1   = uav.UavModel(dt, 'uav_'+name, StatefromSharedPayload('uav_'+name, payload, angSt, lc, j), robot, pload=True, lc=lc)
            if payload.optimize:
                uav1.hyperrpy = robot['hyperplanes']['rpy']
                uav1.hyperyaw = robot['hyperplanes']['yaw']
            j +=3
            uavs['uav_'+name] = uav1    
    else:
        pltrajectory   = params['RobotswithPayload']['payload']['refTrajPath']
        payload_params = {**params['RobotswithPayload']['payload'], 'dt': dt}
        uavs_params    = {}
        for name, robot in params['RobotswithPayload']['Robots'].items():
            trajectories['uav_'+name]   = robot['refTrajPath']
            uavs_params.update({name: {**robot['initConditions'], **robot, 'dt': dt}})
            dt, initState  = initializeState(uavs_params[name])
            uav1           = uav.UavModel(dt, 'uav_'+name, initState, uavs_params[name])
            uavs['uav_'+name] = uav1
        payload_params, payload = setPayloadfromUAVs(uavs_params, payload_params)
    return plStSize, uavs, uavs_params, payload, trajectories, pltrajectory


def initPLController(uavs, payload):
    """This function initializes the controller"""
    states    = {} 
    controls  = {}
    sensors_  = {}
    if payload.ctrlType == 'lee':
        cffirmware.controllerLeePayloadInit() 
    elif payload.ctrlType == 'lee_firmware':
        for id in uavs.keys():
            leePayload = cffirmware.controllerLeePayload_t()
            cffirmware.controllerLeePayloadInit(leePayload)
            leePayload.mp = payload.mp
            leePayload.offsetx = payload.offset[0]
            leePayload.offsety = payload.offset[1]
            leePayload.offsetz = payload.offset[2]
            leePayload.Kpos_P.x = payload.controller['kpx']
            leePayload.Kpos_P.y = payload.controller['kpy']
            leePayload.Kpos_P.z = payload.controller['kpz']
            leePayload.Kpos_D.x = payload.controller['kdx']
            leePayload.Kpos_D.y = payload.controller['kdy']
            leePayload.Kpos_D.z = payload.controller['kdz']
            leePayload.Kpos_I.x = payload.controller['kipx']
            leePayload.Kpos_I.y = payload.controller['kipy']
            leePayload.Kpos_I.z = payload.controller['kipz']

            leePayload.Kprot_P.x = payload.controller['krpx']
            leePayload.Kprot_P.y = payload.controller['krpy']
            leePayload.Kprot_P.z = payload.controller['krpz']
            leePayload.Kprot_D.x = payload.controller['kwpy']
            leePayload.Kprot_D.y = payload.controller['kwpx']
            leePayload.Kprot_D.z = payload.controller['kwpz']

            leePayload.KR.x     = payload.controller['krx']
            leePayload.KR.y     = payload.controller['kry']
            leePayload.KR.z     = payload.controller['krz']
            leePayload.Komega.x = payload.controller['kwx']
            leePayload.Komega.y = payload.controller['kwy']
            leePayload.Komega.z = payload.controller['kwz']
            leePayload.KI.x     = payload.controller['kix']
            leePayload.KI.y     = payload.controller['kiy']
            leePayload.KI.z     = payload.controller['kiz']   

            leePayload.K_q.x    = payload.cablegains['kqx']
            leePayload.K_q.y    = payload.cablegains['kqy']
            leePayload.K_q.z    = payload.cablegains['kqz']
            leePayload.K_w.x    = payload.cablegains['kwcx']
            leePayload.K_w.y    = payload.cablegains['kwcy']
            leePayload.K_w.z    = payload.cablegains['kwcz']
            control = cffirmware.control_t()
            # allocate desired state
            setpoint = cffirmware.setpoint_t()
            setpoint = setTrajmode(setpoint)
            sensors = cffirmware.sensorData_t()
            state = cffirmware.state_t()
            uavs[id].ctrlPayload = leePayload
            states[id] = state
            sensors_[id] = sensors
            controls[id] = control
        return uavs, controls, setpoint, sensors_, states
    for id in uavs.keys():
        cffirmware.controllerLeePayloadInit() 
        control = cffirmware.control_t()
        # allocate desired state
        setpoint = cffirmware.setpoint_t()
        setpoint = setTrajmode(setpoint)
        sensors = cffirmware.sensorData_t()
        state = cffirmware.state_t()
        states[id] = state
        sensors_[id] = sensors
        controls[id] = control
    return controls, setpoint, sensors_, states 

def updateNeighbors(leePayload, state, id, uavs, payload):
    i = 0
    for id_ in uavs.keys():
        if id != id_:
            stateofId = uavs[id_].state
            attPoint = payload.posFrloaddict[id_]
            cffirmware.state_set_neighbor_position(state,  i, stateofId[0], stateofId[1], stateofId[2])
            cffirmware.attPoints_set_neighbors(leePayload, i, attPoint[0], attPoint[1], attPoint[2])
            i+=1
        else:
            attPoint = payload.posFrloaddict[id]
            leePayload.attPoint.x = attPoint[0]
            leePayload.attPoint.y = attPoint[1]
            leePayload.attPoint.z = attPoint[2]
    return leePayload, state

def udpateHpsAndmu(id, uavs, leePayload):
    # This is currently fixed for 3 uavs 2 hyperplanes
    n1 = np.array([leePayload.n1.x, leePayload.n1.y, leePayload.n1.z])
    n2 = np.array([leePayload.n2.x, leePayload.n2.y, leePayload.n2.z])
    n3 = np.array([leePayload.n3.x, leePayload.n3.y, leePayload.n3.z])
    n4 = np.array([leePayload.n4.x, leePayload.n4.y, leePayload.n4.z])
    n5 = np.array([leePayload.n5.x, leePayload.n5.y, leePayload.n5.z])
    n6 = np.array([leePayload.n6.x, leePayload.n6.y, leePayload.n6.z])
    a  = 0
    ids = list(uavs.keys())

    hp1 = hyperplane(n1, a)
    hp2 = hyperplane(n2, a)
    hp3 = hyperplane(n3, a)
    hp4 = hyperplane(n4, a)
    hp5 = hyperplane(n5, a)
    hp6 = hyperplane(n6, a)
    
    uavs[ids[0]].addHp(0 ,hp1)
    uavs[ids[0]].addHp(1 ,hp2)
    
    uavs[ids[1]].addHp(0, hp3)
    uavs[ids[1]].addHp(1, hp4)
    
    uavs[ids[2]].addHp(0, hp5)
    uavs[ids[2]].addHp(1, hp6)

    desVirtInp = leePayload.desVirtInp
    return uavs, desVirtInp

def updatePlstate(state, payload):
    plstate = payload.state
    state.payload_pos.x = plstate[0]   # m
    state.payload_pos.y = plstate[1]    # m
    state.payload_pos.z = plstate[2]    # m
    state.payload_vel.x = plstate[3]    # m/s
    state.payload_vel.y = plstate[4]    # m/s
    state.payload_vel.z = plstate[5]    # m/s
    if not payload.pointmass:
        q_curr = np.array(plstate[6:10]).reshape((4,))
        state.payload_quat.w = q_curr[0]
        state.payload_quat.x = q_curr[1]
        state.payload_quat.y = q_curr[2]
        state.payload_quat.z = q_curr[3]
        state.payload_omega.x = plstate[10]
        state.payload_omega.y = plstate[11]
        state.payload_omega.z = plstate[12]
    return state


def updatePlDesState(setpoint, payload, fulltraj):
    setpoint.position.x = fulltraj[0]  # m
    setpoint.position.y = fulltraj[1]  # m
    setpoint.position.z = fulltraj[2]  # m
    setpoint.velocity.x = fulltraj[3]  # m/s
    setpoint.velocity.y = fulltraj[4]  # m/s
    setpoint.velocity.z = fulltraj[5]  # m/s
    setpoint.acceleration.x = fulltraj[6]  # m/s^2
    setpoint.acceleration.y = fulltraj[7]  # m/s^2
    setpoint.acceleration.z = fulltraj[8]  # m/s^2
    if not payload.pointmass:
        attSetpoint = payload.attSetpoint
        q_des = rn.from_euler(np.radians(attSetpoint[0]), np.radians(attSetpoint[1]), np.radians(attSetpoint[2]), convention="xyz", axis_type="extrinsic")
        setpoint.attitudeQuaternion.w = q_des[0]
        setpoint.attitudeQuaternion.x = q_des[1]
        setpoint.attitudeQuaternion.y = q_des[2]
        setpoint.attitudeQuaternion.z = q_des[3]
        setpoint.attitudeRate.roll  = 0
        setpoint.attitudeRate.pitch = 0
        setpoint.attitudeRate.yaw   = 0
        
    if len(fulltraj) == 15:
        setpoint.jerk.x = fulltraj[9]
        setpoint.jerk.y = fulltraj[10]
        setpoint.jerk.z = fulltraj[11]
        if payload.ctrlType == 'lee':
            setpoint.snap.x = fulltraj[12]
            setpoint.snap.y = fulltraj[13]
            setpoint.snap.z = fulltraj[14]
    elif len(fulltraj) == 9:
        setpoint.jerk.x = 0 
        setpoint.jerk.y = 0 
        setpoint.jerk.z = 0 
        if payload.ctrlType == 'lee':
            setpoint.snap.x = 0 
            setpoint.snap.y = 0 
            setpoint.snap.z = 0 
    return setpoint

def setPlanes(leePayload, rpyplanes4robots, yaw4robots):
    leePayload.rpyPlane11.x = rpyplanes4robots[0][0][0]
    leePayload.rpyPlane11.y = rpyplanes4robots[0][0][1]
    leePayload.rpyPlane11.z = rpyplanes4robots[0][0][2]
    leePayload.rpyPlane12.x = rpyplanes4robots[0][1][0]
    leePayload.rpyPlane12.y = rpyplanes4robots[0][1][1]
    leePayload.rpyPlane12.z = rpyplanes4robots[0][1][2]
    leePayload.rpyPlane21.x = rpyplanes4robots[1][0][0]
    leePayload.rpyPlane21.y = rpyplanes4robots[1][0][1]
    leePayload.rpyPlane21.z = rpyplanes4robots[1][0][2]
    leePayload.rpyPlane22.x = rpyplanes4robots[1][1][0]
    leePayload.rpyPlane22.y = rpyplanes4robots[1][1][1]
    leePayload.rpyPlane22.z = rpyplanes4robots[1][1][2]
    leePayload.rpyPlane31.x = rpyplanes4robots[2][0][0]
    leePayload.rpyPlane31.y = rpyplanes4robots[2][0][1]
    leePayload.rpyPlane31.z = rpyplanes4robots[2][0][2]
    leePayload.rpyPlane32.x = rpyplanes4robots[2][1][0]
    leePayload.rpyPlane32.y = rpyplanes4robots[2][1][1]
    leePayload.rpyPlane32.z = rpyplanes4robots[2][1][2]
    leePayload.yawPlane11   = yaw4robots[0][0][0]
    leePayload.yawPlane12   = yaw4robots[0][1][0]
    leePayload.yawPlane21   = yaw4robots[1][0][0]
    leePayload.yawPlane22   = yaw4robots[1][1][0]
    leePayload.yawPlane31   = yaw4robots[2][0][0]
    leePayload.yawPlane32   = yaw4robots[2][1][0]
    return leePayload

##----------------------------------------------------------------------------------------------------------------------------------------------------------------##        
##----------------------------------------------------------------------------------------------------------------------------------------------------------------##
def main(args, animateOrPlotdict, params):
    # Initialize an instance of a uav dynamic model with:
    # dt: time interval
    # initState: initial state
    # set it as 1 tick: i.e: 1 ms
    # pload: payload flag, enabled: with payload, otherwise: no payload 
    filename = args.filename
    initUavs = args.initUavs
    simtime  = float(params['simtime'])
    sample   = int(params['sample'])
    shared = False
    if params['RobotswithPayload']['payload']['mode'] in 'shared':
       plStSize, uavs, uavs_params, payload, trajectories, pltrajectory = setTeamParams(params, initUavs)
       shared = True
    else:
        uavs, payload, trajectories = setParams(params)
    # Upload the traj in csv file format
    # rows: time, xdes, ydes, zdes, vxdes, vydes, vzdes, axdes, aydes, azdes  
    timeStamped_traj = {}
    if not uavs:
         sys.exit('no UAVs')

    if shared and payload.lead:
        input = pltrajectory
        timeStamped_traj = np.loadtxt(input, delimiter=',') 
        tf_ms = timeStamped_traj[0,-1]*1e3
    else:
        for id in uavs.keys():
            input = trajectories[id]
            timeStamped_traj[id] = np.loadtxt(input, delimiter=',') 
            tf_ms = timeStamped_traj[id][0,-1]*1e3
    # Simulation time
    tf_sim = tf_ms + simtime
    # final time of traj in ms
    print('\nTotal Simulation time: '+str(tf_sim*1e-3)+ 's')
    print('Trajectory duration: '+str(tf_ms*1e-3)+ 's\n')
    print()
    print('UAVs Initial States: [x y z xdot ydot zdot qw qx qy qz wx wy wz] \n')
    for key in uavs.keys():
        print(key, uavs[key].state)
    if shared:
        print() 
        print('payload pos:',payload.state[0:3])

    print()
    print('Simulating...')

    if shared:
        if payload.lead:
            if payload.ctrlType == 'lee_firmware':
                uavs, controls, setpoint, sensors_, states = initPLController(uavs, payload)
                ids = list(uavs.keys())
                pairsinIds = list(permutations(ids, 2))
                Kp    = np.diag([0,0,0])
                Kd    = np.diag([0,0,0])
                floor = uav.environment(Kp, Kd, np.array([0, 0, 0]))
                ids = list(uavs.keys())
                allPairs = {}
                for i in ids:
                    idstmp = ids.copy()
                    idstmp.remove(i)
                    allPairs[i] = idstmp
                # prepare hyperplanes in a list for all UAVs to use them
                if payload.optimize:         
                    rpyplanes4robots = []
                    yaw4robots = []
                    for id in uavs.keys():
                        rpyplanes4robots.append(uavs[id].hyperrpy)
                        yaw4robots.append(uavs[id].hyperyaw)    
            elif payload.ctrlType == 'lee':
                Kp    = np.diag([0,0,0])
                Kd    = np.diag([0,0,0])
                floor = uav.environment(Kp, Kd, np.array([0, 0, 0]))
                controls, setpoint, sensors_, states = initPLController(uavs, payload)
                ids = list(uavs.keys())
                allPairs = {}
                for i in ids:
                    idstmp = ids.copy()
                    idstmp.remove(i)
                    allPairs[i] = idstmp
        else:
            controls, setpoints, sensors_, states, lees =  {}, {}, {}, {}, {} 
            for id in uavs.keys():
                if uavs[id].controller['name'] == 'lee_firmware':
                    lee, control, setpoint, sensors, state = initController(uavs[id].controller)
                    lees[id] = lee
                else:
                    control, setpoint, sensors, state = initController(uavs[id].controller)
                controls[id]  = control
                setpoints[id] = setpoint
                sensors_[id]  = sensors
                states[id]    = state 
         # fullCtrlInps = []
        for tick in range(0, int(tf_sim)+1):
            j = plStSize
            ctrlInputs = np.zeros((1,4))
            if payload.lead:
                ## Update setpoint of payload desired states
                if tick <= int(tf_ms):   
                    setpoint  = updatePlDesState(setpoint, payload, timeStamped_traj[1::,tick])
                    plref_state   = np.array([setpoint.position.x, setpoint.position.y, setpoint.position.z, setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z])
                else: 
                    setpoint  = updatePlDesState(setpoint, payload, timeStamped_traj[1::,-1])
                    plref_state = np.array([setpoint.position.x, setpoint.position.y, setpoint.position.z, setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z])
            try:
                ## Update states and  control input for each UAV
                desVirtInp = []
                for id in uavs.keys():
                    if not payload.lead:
                        control, setpoint, sensors, state = controls[id], setpoints[id], sensors_[id], states[id]
                    elif payload.lead: 
                        control, sensors, state = controls[id], sensors_[id], states[id]

                    #initialize the controller and allocate current state (both sensor and state are the state)
                    # This is kind of odd and should be part of state
                    if tick <= int(tf_ms):
                        if not payload.lead:    
                            setpoint  = updateDesState(setpoint, uavs[id].controller, timeStamped_traj[id][1::,tick])
                            ref_state = np.array(timeStamped_traj[id][1:7,tick])
                        else: 
                            ref_state = uavs[id].state[0:6]
                    else:
                        if not payload.lead:    
                            setpoint  = updateDesState(setpoint, uavs[id].controller, timeStamped_traj[id][1::,-1])
                            ref_state = np.array(timeStamped_traj[id][1:7,-1])
                        else:
                            ref_state =  uavs[id].state[0:6]
                    # update current state              
                    # update the state of the payload 
                    state   =  updatePlstate(state, payload)
                    state, fullState = updateState(state, uavs[id])

                    sensors  = updateSensor(sensors, uavs[id])

                    if payload.lead:
                        ## Choose controller: Python or firmware
                        if payload.ctrlType == 'lee': # Python
                            try:
                                uavs, payload, control, des_w, des_wd = cffirmware.controllerLeePayload(uavs, id, payload, control, setpoint, sensors, state, tick, j)
                                ref_state = np.append(ref_state, np.array([des_w, des_wd]).reshape(6,), axis=0)
                            except Exception as e:
                                print('tick: ',tick)
                                print(f"Controller failed, Unexpected {e=}, {type(e)=}")
                                print('Error on line {}'.format(sys.exc_info()[-1].tb_lineno), type(e).__name__, e)
                                print()
                                raise
                                break

                        elif payload.ctrlType == 'lee_firmware': # Firmware
                            leePayload = uavs[id].ctrlPayload
                            leePayload.mass = uavs[id].m
                            try:
                                leePayload, state = updateNeighbors(leePayload, state, id, uavs, payload)
                                cffirmware.controllerLeePayload(leePayload, control, setpoint, sensors, state, tick)
                                uavs, desVirtInp_i = udpateHpsAndmu(id, uavs, leePayload)
                                desVirtInp.append(desVirtInp_i)
                            except Exception as e:
                                print('tick: ',tick)
                                print(f"Controller failed, Unexpected {e=}, {type(e)=}")
                                print('Error on line {}'.format(sys.exc_info()[-1].tb_lineno), type(e).__name__, e)
                                print()
                                raise
                                break
                            des_w, des_wd  = np.zeros(3,), np.zeros(3,)
                            ref_state = np.append(ref_state, np.array([des_w, des_wd]).reshape(6,), axis=0)
                    else:
                        if uavs[id].controller['name'] == 'lee':
                            control, des_w, des_wd  = cffirmware.controllerLee(uavs[id], control, setpoint, sensors, state, tick)
                            ref_state = np.append(ref_state, np.array([des_w, des_wd]).reshape(6,), axis=0)     
                        elif uavs[id].controller['name'] == 'lee_firmware':
                            lee = lees[id]
                            cffirmware.controllerLee(lee, control, setpoint, sensors, state, tick)      
                            des_w, des_wd  = np.zeros(3,), np.zeros(3,)
                            ref_state = np.append(ref_state, np.array([des_w, des_wd]).reshape(6,), axis=0)
                        else:    
                            cffirmware.controllerSJC(control, setpoint, sensors, state, tick)    
                            des_w, des_wd  = np.zeros(3,), np.zeros(3,)
                            ref_state = np.append(ref_state, np.array([des_w, des_wd]).reshape(6,), axis=0)        
                    
                    control_inp    = np.array([control.thrustSI, control.torque[0], control.torque[1], control.torque[2]])
                    ctrlInp        = np.array([control.u_all[0], control.u_all[1], control.u_all[2]])
                    # fullCtrlInps.append(list(ctrlInp))
                    uavs[id].state = StatefromSharedPayload(id, payload, uavs[id].state[6::], uavs[id].lc, j)
                    ctrlInputs     = np.vstack((ctrlInputs, control_inp.reshape(1,4)))
                    payload.stackCtrl(ctrlInp.reshape(1,3))  
                    if not payload.lead:
                        controls[id]  = control
                        setpoints[id] = setpoint
                        sensors_[id]  = sensors
                        states[id]    = state
                    elif payload.lead:
                        controls[id]  = control
                        sensors_[id]  = sensors
                        states[id]    = state
                    uavs[id].stackStandCtrl(uavs[id].state, control_inp, ref_state)
                    j+=3
                if payload.ctrlType == 'lee_firmware':
                    payload.mu_des_prev = np.array(desVirtInp).reshape((3*payload.numOfquads,))
                payload.stackmuDes(payload.mu_des_prev)
                payload.cursorUp() 
                # Evolve the payload states
                uavs, loadState =  payload.stateEvolution(ctrlInputs, uavs, uavs_params, floor.interactionForce(payload.state[0:3], payload.state[3:6]))    
                if payload.lead:
                    payload.stackStateandRef(plref_state)
                else:
                    payload.stackState()     
            except Exception as e:
                print(f"Simulation failed in {e=}, {type(e)=}")
                print('Error on line {}'.format(sys.exc_info()[-1].tb_lineno), type(e).__name__, e)
                print()
                break 
                raise

        for id in uavs.keys():
            uavs[id].cursorUp()
            uavs[id].removeEmptyRow()
        
        payload.cursorPlUp()
        payload.removemu()
        splitStackMu = np.hsplit(payload.mu_des_stack,payload.numOfquads)
        hpsDict = {}
        mufilePathsperId = []
        muDict = {}
        stDict = {}

        # Payload csv file
        with open("output/payload.csv", "w") as f:
            np.savetxt(f, payload.plFullState, delimiter=",")

        # mu per robot csv file
        for id, stackMu in zip(uavs.keys(), range(len(splitStackMu))):
            mufilePathsperId = []
            uavID = id.replace("uav_", "")
            fName = "mu_" + uavID + ".csv"
            mufilepath = "output/"+ fName           
            with open(mufilepath, "w") as f:
                np.savetxt(f, splitStackMu[stackMu], delimiter=",")
            
            muDict[uavID] = fName
    
        # state per robot csv
        for id in uavs.keys():
            uavID = id.replace("uav_", "")
            fName = uavID + ".csv"

            with open("output/" + fName, "w") as f:
                np.savetxt(f, uavs[id].fullState, delimiter=",")    
            stDict[uavID] = fName
            # hps per robot csv
            hpsfilePathsperId = []           
            for hpPerId in uavs[id].hpStack.keys(): 
                fName = "hp" + str(hpPerId+1) + "_" + uavID + ".csv"               
                hpfilepath = "output/" + fName
                hpsfilePathsperId.append(fName)
                with open(hpfilepath, "w") as f:
                    np.savetxt(f, uavs[id].hpStack[hpPerId][::payload.numOfquads,:], delimiter=",")
       
            hpsDict[uavID] = hpsfilePathsperId

        ## write the config file for visualization
        configData = {}
        configData['robots'] = {}
        configData['payload'] = 'payload.csv'
        Ids = []
        for id in uavs.keys():
            uavID = id.replace("uav_", "")
            Ids.append(uavID)
    
        for id in Ids:
            robot        = {}
            robot['state'] = stDict[id]
            robot['hps']   = hpsDict[id]
            robot['mu']    = muDict[id]
            if not payload.pointmass:
                att = payload.posFrloaddict['uav_'+id]
                robot['att']   = att.tolist()
            configData['robots'][id] = robot
        with open("output/configData.yaml", 'w') as f:
            yaml.dump(configData, f)
        
        ## Animate or plot based on flags
        animateOrPlot(uavs, payload, animateOrPlotdict, filename, tf_sim, shared, sample)
        # return uavs, payload, fullCtrlInps, u_par, u_per
    else:
        for id in uavs.keys():
            #initialize the controller and allocate current state (both sensor and state are the state)
            # This is kind of odd and should be part of state
            if uavs[id].controller['name'] == 'lee_firmware':
                lee, control, setpoint, sensors, state = initController(uavs[id].controller)
            else:
                control, setpoint, sensors, state = initController(uavs[id].controller)
            # Note that 1 tick == 1ms
            # note that the attitude controller will only compute a new output at 500 Hz
            # and the position controller only at 100 Hz
            # If you want an output always, simply select tick==0
            # if uavs[id].pload:
            #     payload = payload[id]
            
            for tick in range(0, int(tf_sim)+1):
                # update desired state
                if tick <= int(tf_ms):    
                    setpoint  = updateDesState(setpoint, uavs[id].controller, timeStamped_traj[id][1::,tick])
                    ref_state =  np.array(timeStamped_traj[id][1:7,tick])
                    
                else:
                    setpoint  = updateDesState(setpoint, uavs[id].controller, timeStamped_traj[id][1::,-1])
                    ref_state = np.array(timeStamped_traj[id][1:7,-1])
                # update current state
                state,fullState = updateState(state, uavs[id])
                sensors         = updateSensor(sensors, uavs[id])
                # query the controller
                if uavs[id].controller['name'] in 'lee':
                    control, des_w, des_wd  = cffirmware.controllerLee(uavs[id], control, setpoint, sensors, state, tick)     
                    ref_state = np.append(ref_state, np.array([des_w, des_wd]).reshape(6,), axis=0)     
                elif uavs[id].controller['name'] in 'lee_firmware':
                    cffirmware.controllerLee(lee, control, setpoint, sensors, state, tick)   
                    des_w, des_wd  = np.zeros(3,), np.zeros(3,)
                    ref_state = np.append(ref_state, np.array([des_w, des_wd]).reshape(6,), axis=0)                        
                else:    
                    cffirmware.controllerSJC(control, setpoint, sensors, state, tick)  
                    des_w, des_wd  = np.zeros(3,), np.zeros(3,)
                    ref_state = np.append(ref_state, np.array([des_w, des_wd]).reshape(6,), axis=0)             
                control_inp = np.array([control.thrustSI, control.torque[0], control.torque[1], control.torque[2]])
                if uavs[id].pload:
                    payload[id].PL_nextState(control_inp, uavs[id])
                else:
                    uavs[id].states_evolution(control_inp)  # states evolution
                uavs[id].stackStandCtrl(uavs[id].state, control_inp, ref_state)    
            uavs[id].cursorUp()
            if uavs[id].pload:
                payload[id].cursorUp()
                
        # Animation        
        animateOrPlot(uavs, payload, animateOrPlotdict, filename, tf_sim, shared, sample)    


if __name__ == '__main__':
    try: 
        parser = argparse.ArgumentParser()
        parser.add_argument('filename', type=str, help="Name of the CSV file in trajectoriescsv directory")
        parser.add_argument('--animate', default=False, action='store_true', help='Set true to save a gif in Videos directory')
        parser.add_argument('--plot', default=False, action='store_true', help='Set true to save plots in a pdf  format')
        parser.add_argument('--initUavs', default=False, action='store_true', help='Set true to initialize the conditions of the UAVs and then compute the payload initial condition')
        args   = parser.parse_args()   
        animateOrPlotdict = {'animate':args.animate, 'plot':args.plot}
    
        with open('config/initialize.yaml') as f:
            params = yaml.load(f, Loader=yaml.FullLoader)
        main(args, animateOrPlotdict, params)
    except ImportError as imp:
        print(imp)
