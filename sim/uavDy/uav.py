import numpy as np
from rowan.calculus import integrate as quat_integrate
from rowan.functions import _promote_vec, _validate_unit, exp, multiply
from rowan import from_matrix, to_matrix, to_euler, from_euler, rotate
from scipy import  integrate, linalg
from numpy.polynomial import Polynomial as poly
import sys
import time

def skew(w):
    w = w.reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))

class environment:
    def __init__(self, Kp, Kd, pd):
        self.Kp = Kp
        self.Kd = Kd
        self.pd = pd
        self.vd = np.zeros(3,)

    def interactionForce(self, p, v):
        F = np.zeros(3,)
        if p[2] <= self.pd[2]:
            self.pd[0:2] = p[0:2]
            ep = self.pd - p
            ed = self.vd - v
            F =  self.Kp@ep + self.Kd@ed 
            F = np.clip(F, np.array([-0.5,-0.5, -0.5]), np.array([0.5, 0.5, 0.5]))
        return F

class Payload:
    def __init__(self, dt, state, params):
        self.mp      = float(params['m_p']) # Mass of payload [kg]
        self.lc      = float(params['l_c'])  # length of cable [m]
        self.m       = float(params['m']) # Mass of quadrotor [kg]
        self.mt      = self.m + self.mp # Total mass [kg]
        self.grav_   = np.array([0,0,-self.mt*9.81])
        # state = [xl, yl, zl, xldot, yldot, zldot, px, py, pz, wlx, wly, wlz, qw, qx, qy, qz, wx, wy, wz]
        self.state   = state
        self.dt      = dt

        self.plFullState = np.empty((1,19))
    def __str__(self):
        return "payload m = {} kg, length of cable = {} m, \n\n Initial State = {}".format(self.mp, self.lc, self.state)

    def getPL_nextpos(self, fz, curr_posl, curr_vl, curr_p, curr_wl, curr_q):
        R_IB  = to_matrix(curr_q)
        pd    = np.cross(curr_wl, curr_p)
        u     = fz * R_IB * np.array([0,0,1]) 
        self.al    =  (1/self.mt) * (self.grav_ + (np.vdot(curr_p, R_IB @ np.array([0,0,fz])) - (self.m * self.lc * (np.vdot(pd, pd)))) * curr_p)
        Vl_   = self.al * self.dt + curr_vl
        posl_ = curr_vl * self.dt + curr_posl
        return posl_, Vl_

    def getPLAngularState(self, fz, curr_q, curr_p, curr_wl):
        R_IB = to_matrix(curr_q)
        wld  = (1/(self.lc*self.m)) * ( skew(-curr_p) @ R_IB @ np.array([0,0,fz]))
        wl_  = wld * self.dt + curr_wl
        pd    =  skew(curr_wl) @ curr_p
        p_    = pd*self.dt + curr_p
        return p_, wl_

    def PL_nextState(self, control_t, uav):
        curr_posl = self.state[0:3]   # position: x,y,z
        curr_vl   = self.state[3:6]   # linear velocity: xdot, ydot, zdot
        curr_p    = self.state[6:9]   # directional unit vector
        curr_wl   = self.state[9:12]  # Payload angular velocity in Inertial Frame
        curr_q    = self.state[12:16] # Quaternions: [qw, qx, qy, qz]
        curr_w    = self.state[16::]  # Quadrotor angular velocity

        fz       = control_t[0]
        tau_i    = control_t[1::]
        control_t[0] -= self.mp * 9.81
        uavState = uav.states_evolution(control_t)
        qNext = uav.state[6:10]
        wNext = uav.state[10::]

        poslNext, VlNext  = self.getPL_nextpos(fz, curr_posl, curr_vl, curr_p, curr_wl, curr_q)
        pNext, wlNext     = self.getPLAngularState(fz, curr_q, curr_p, curr_wl) 
        self.state[0:3]   = poslNext   # position: x,y,z
        self.state[3:6]   = VlNext  # linear velocity: xdot, ydot, zdot
        self.state[6:9]   = pNext # directional unit vector
        self.state[9:12]  = wlNext # Payload angular velocity in Inertial Frame
        self.state[12:16] = qNext # Quadrotor attitude [q = qw, qx, qy, qz]
        self.state[16::]  = wNext # Quadrotor angular velocity [w = wx, wy, wz]
        self.plFullState  = np.vstack((self.plFullState, self.state))

    def cursorUp(self):
        ## This method removes the first row of the stack which is initialized as an empty array
        self.plFullState = np.delete(self.plFullState, 0, 0)

class SharedPayload:
    def __init__(self, payload_params, uavs_params):
        self.dt = payload_params['dt']
        self.g  = 9.81
        self.mp = float(payload_params['m_p']) # Mass of payload [kg]
        self.J = np.diag(payload_params['inertia'])
        self.mt_  = 0
        self.numOfquads = 0
        if payload_params['payloadCtrl']['payloadLead'] == 'enabled':
            self.lead = True
        else: 
            self.lead = False
        self.controller    = payload_params['payloadCtrl']['gains']['ctrlLee']
        self.cablegains    = payload_params['payloadCtrl']['gains']['cable']
        self.ctrlType      = payload_params['payloadCtrl']['name']
        self.offset        = np.array(payload_params['payloadCtrl']['offset'])
        self.optimize      = payload_params['payloadCtrl']['optimize']['mode']
        self.qp_tool       = payload_params['payloadCtrl']['optimize']['qp']
        self.downwashAware = payload_params['payloadCtrl']['optimize']['downwashAware']
        self.lambdaa = payload_params['payloadCtrl']['optimize']['lambda']
        self.attSetpoint   = payload_params['refAttitude']
        self.shape = payload_params['shape']
        self.gen_hp = payload_params['payloadCtrl']['gen_hp']
        self.en_qdidot = payload_params['payloadCtrl']['en_qdidot']
        self.lambda_svm = payload_params['payloadCtrl']['lambda_svm']
        self.en_accrb = payload_params['payloadCtrl']['en_accrb']
        self.loadangles = payload_params['payloadCtrl']['optimize']['loadangles']
        self.load = payload_params['payloadCtrl']['optimize']['load']
        self.desiredFormation = payload_params['payloadCtrl']['optimize']['qi_des']
        self.desFormFlag = payload_params['payloadCtrl']['optimize']['desFormation']
        self.posFrload = np.empty((1,3))
        self.posFrloaddict = {}
        self.Fd = []
        self.Md = []
        if self.optimize == 'enabled':
            self.optimize == True
        else: 
            self.optimize = False
        for name, uav in uavs_params.items():
            self.posFrload = np.vstack((self.posFrload, np.array(uav['pos_fr_payload']).reshape((1,3))))
            self.mt_   += float(uav['m']) # Mass of quadrotors [kg] 
            self.numOfquads += 1   
        self.mt    = self.mp + self.mt_ # total mass of quadrotors and payload
        self.grav_ = np.array([0,0,-self.mt*self.g])
        #state = [xp, yp, zp, xpd, ypd, zpd, qwp, qxp, qyp, qzp, wpx, wpy, wpz, q1,...,qn, w1,...,wn]
        self.plSysDim    = 6
        self.plStateSize = 13
        self.pointmass   = False

        if np.linalg.det(self.J) == 0:
            self.plSysDim -= 3
            self.plStateSize -= 7
            self.pointmass = True
        self.posFrload = np.delete(self.posFrload, 0, 0)
        self.qdi_prevdict  = {}
        self.wdi_prevdict  = {}
        z = 0
        for name in uavs_params.keys():
            self.posFrloaddict['uav_'+name] = self.posFrload[z,:]
            self.qdi_prevdict['uav_'+name] = np.array([0,0,-1])
            self.wdi_prevdict['uav_'+name] = np.array([0,0,0])
            z+=1
        self.sys_dim    = self.plSysDim + 3*self.numOfquads
        self.state_size = self.plStateSize + 6*self.numOfquads #13 for the payload and (3+3)*n for each cable angle and its derivative    

        self.plstate = np.empty((1,self.state_size))
        self.plFullState = np.empty((1,self.state_size))
        self.ctrlInp = np.empty((1,3))
        self.plref_state = np.empty((1,6))
        self.state, self.prevSt = self.getInitState(uavs_params, payload_params)
        self.accl   = np.zeros(self.sys_dim,)
        self.accl_prev = self.accl
        self.i_error = np.zeros(3,)
        self.qdi_prev = np.array([0,0,-1])
        self.wdi_prev = np.array([0,0,0])
        self.mu_des_prev = np.zeros(3*self.numOfquads,)
        self.mu_des_stack = np.empty(3*self.numOfquads,)
        self.ui_s = {}

    def getInitState(self, uav_params, payload_params):
        self.state = np.zeros(self.state_size,)
        self.accl   = np.zeros(self.sys_dim,)
        self.accl[0:3] = np.array([0,0,0]) 
        self.state[3:6]   = self.accl[0:3]*self.dt + payload_params['init_linV_L']
        self.state[0:3]   = self.state[3:6]*self.dt + payload_params['init_pos_L']
        if not self.pointmass:
            init_ang     = np.radians(payload_params['init_angle']) 
            self.state[6:10]  = from_euler(init_ang[0], init_ang[1], init_ang[2])
            self.state[10:13] = payload_params['wl']
        j = self.plStateSize
        for initValues in uav_params.values():
            angR      = np.radians(initValues['q_dg'])
            self.state[j:j+3] = to_matrix(from_euler(angR[0], angR[1], angR[2], convention='xyz',axis_type='extrinsic')) @ np.array([0,0,-1])
            self.state[j+3*self.numOfquads:j+3+3*self.numOfquads] = initValues['qd']
            j+=3
        ctrlInp = np.empty((self.numOfquads,3))
        self.prevSt = self.state.copy()
        return self.state, self.prevSt

    def getBq(self, uavs_params):
        Bq = np.zeros((self.sys_dim, self.sys_dim))
        Bq[0:3,0:3] = self.mt*np.identity(3)
       
        i = self.plSysDim
        k = self.plStateSize
        for name, uav in uavs_params.items():
            m = float(uav['m'])
            l = float(uav['l_c'])
            qi = self.state[k:k+3]
            k+=3
            Bq[i:i+3,0:3]    = -m*skew(qi) # Lee 2018
            Bq[i:i+3, i:i+3] = m*(l)*np.eye(3) # Lee 2018

            if not self.pointmass:
                R_p = to_matrix(self.state[6:10])
                posFrload = np.array(uav['pos_fr_payload'])
                qiqiT = qi.reshape((3,1))@(qi.T).reshape((1,3))
                Bq[0:3, 3:6]   += -m * qiqiT @ R_p @ skew(posFrload)
                Bq[3:6, 0:3]   +=  m * skew(posFrload) @ R_p.T @ qiqiT
                Bq[3:6, 3:6]   +=  m * skew(posFrload) @ R_p.T @ qiqiT @ R_p @ skew(posFrload)
                Bq[i:i+3, 3:6]  =  m * skew(qi) @ R_p @ skew(posFrload) 
            i+=3
        if not self.pointmass:
            Bq[3:6, 3:6] = self.J - Bq[3:6, 3:6]
        return Bq

    def getNq(self, uavs_params):
        Nq =  np.zeros((self.sys_dim,))
        i = self.plSysDim
        k = self.plStateSize
        Mq   = self.mt*np.identity(3)

        if not self.pointmass:
            R_p = to_matrix(self.state[6:10])
            wl = self.state[10:13]            
           
        for name, uav in uavs_params.items():
            m = float(uav['m'])
            l = float(uav['l_c'])
            
            qi = self.state[k:k+3]
            wi = self.state[k+3*self.numOfquads:k+3+3*self.numOfquads]
            k+=3

            if self.pointmass:
                Nq[0:3]  += -m*l*np.dot(wi,wi)*qi # Lee 2018
            else:
                posFrload = np.array(uav['pos_fr_payload'])
                qiqiT = qi.reshape((3,1))@(qi.T).reshape((1,3))
                Nq[0:3]   += (-m*l*np.dot(wi,wi)*qi - m * qiqiT @ R_p @skew(wl) @skew(wl) @ posFrload)
                Nq[3:6]   += skew(posFrload) @ R_p.T @ ((-m *l * np.dot(wi, wi) * qi) - (m * qiqiT @ R_p @ skew(wl) @skew(wl) @ posFrload))

                Nq[i:i+3] = m*skew(qi) @ R_p @ skew(wl) @ skew(wl) @ posFrload  
            i+=3

        if not self.pointmass:
            Nq[3:6] = Nq[3:6] - skew(wl) @ self.J @ wl
        return Nq

    def getuinp(self, uavs_params, ctrlInputs, uavs):        
        u_inp = np.zeros((self.sys_dim,))
        i, j, k = 0, self.plSysDim, self.plStateSize

        for name, uav in uavs_params.items():
            m = float(uav['m'])
            l = float(uav['l_c'])
            f = ctrlInputs[i,0]
            u_i = rotate(uavs["uav_"+name].state[6:10], np.array([0,0,f]))
            if not self.pointmass:
                R_p = to_matrix(self.state[6:10])
                wl = self.state[10:13]
                posFrload = np.array(uav['pos_fr_payload'])
     
            qi = self.state[k:k+3]
            wi = self.state[k+3*self.numOfquads:k+3+3*self.numOfquads]
            k+=3
            
            qiqiT = qi.reshape((3,1))@(qi.T).reshape((1,3))
            u_inp[0:3] += u_i
            
            u_perp = ((np.eye(3) - qiqiT) @  u_i)
            u_inp[j:j+3] = -skew(qi) @ u_i

            if not self.pointmass:
                u_inp[3:6] += skew(posFrload)@R_p.T @ qiqiT @ u_i
            i+=1
            j+=3
        return u_inp

    def getNextState(self):
        # if not pointmass:
            #state = [xp, yp, zp, xpd, ypd, zpd, qwp, qxp, qyp, qzp, wpx, wpy, wpz, q1,...,qn, w1,..,wn]
        #else:
            #state = [xp, yp, zp, xpd, ypd, zpd, q1,...,qn, w1,...,wn]
        if not self.pointmass:
            posNext = np.zeros(self.sys_dim+1)
        xp  = self.state[0:3]
        vp  = self.state[3:6]
        if not self.pointmass:
            quat_p = self.state[6:10]
            wp = self.state[10:13]  
            self.state[10:13] = self.accl[3:6]*self.dt + wp #wp_next
            self.state[6:10] = self.integrate_quat(quat_p, wp, self.dt) # payload quaternion (atttitude)   

        self.state[0:3] = vp * self.dt + xp  #xp_next        
        self.state[3:6] = self.accl[0:3] * self.dt + vp #vp_next
        k = self.plStateSize        
        j = self.plSysDim
        for i in range(0, self.numOfquads):
            qi = self.state[k:k+3]
            wi = self.state[k+3*self.numOfquads:k+3+3*self.numOfquads]
            wdi = self.accl[j:j+3]
            self.state[k+3*self.numOfquads:k+3+3*self.numOfquads] = wdi*self.dt + wi # wi_next: cables angular velocity
            qdot = np.cross(wi, qi) # qdot 
            self.state[k:k+3] = qdot*self.dt + qi # qi_next: cables directional vectors
            k+=3
            j+=3

    def stateEvolution(self, ctrlInputs, uavs, uavs_params):
        ctrlInputs = np.delete(ctrlInputs, 0,0)
        Bq    = self.getBq(uavs_params)
        Nq    = self.getNq(uavs_params)
        u_inp = self.getuinp(uavs_params, ctrlInputs, uavs)
        
        k = self.plStateSize
        j = self.plSysDim
        self.plstate[0,0:3] = self.state[0:3]
        self.plstate[0,3:6] = self.state[3:6]
        self.plstate[0,6:10] = self.state[6:10]
        for i in range(0, self.numOfquads):
            self.plstate[0,k:k+3] = self.state[k:k+3]
            self.plstate[0,k+3*self.numOfquads:k+3+3*self.numOfquads] = self.state[k+3*self.numOfquads:k+3+3*self.numOfquads]
            k+=3
            j+=3
        try:
            self.accl = np.linalg.inv(Bq)@(Nq + u_inp)
            self.accl[0:3] -= np.array([0,0,9.81])
            self.accl_prev = self.accl
        except Exception as err:
            print(f"Unexpected {err=}, {type(err)=}")
            self.accl = self.accl_prev
            raise
        self.prevSt = self.state.copy()
        # print(self.prevSt[0:3])
        self.getNextState()
        # print(self.state[0:3])
        self.ctrlInp = np.zeros((1,3))

        m, k = 0 , self.plStateSize
        for id in uavs.keys():
            tau = ctrlInputs[m,1::].reshape(3,)
            curr_q = uavs[id].state[6:10]
            curr_w = uavs[id].state[10::]
            qNext, wNext = uavs[id].getNextAngularState(curr_w, curr_q, tau)
            uavs[id].state[6:10] = qNext
            uavs[id].state[10::] = wNext
            m+=1
        return uavs, self.state 
    
    def stackmuDes(self, mu_des):
        self.mu_des_stack = np.vstack((self.mu_des_stack, mu_des))

    def stackCtrl(self, ctrlInp):  
       self.ctrlInp = np.vstack((self.ctrlInp,ctrlInp))
    
    def stackState(self):
        self.plFullState = np.vstack((self.plFullState, self.state)) 
    
    def stackStateandRef(self,plref_state):
        self.plFullState = np.vstack((self.plFullState, self.state)) 
        self.plref_state = np.vstack((self.plref_state, plref_state.reshape((1,6)))) 

    def removemu(self):
        self.mu_des_stack = np.delete(self.mu_des_stack, 0, 0)
         
    def cursorPlUp(self):
        self.plFullState = np.delete(self.plFullState, 0, 0)
        self.plref_state = np.delete(self.plref_state, 0, 0)

    def cursorUp(self):
        self.ctrlInp = np.delete(self.ctrlInp, 0, 0)

    def integrate_quat(self, q, wb, dt):
        return multiply(q, exp(_promote_vec(wb * dt / 2))) 
      

class UavModel:
    """initialize an instance of UAV object with the following physical parameters:
    m = 0.034 [kg]  -------------------------------------> Mass of the UAV
    I =   (16.571710 0.830806 0.718277
            0.830806 16.655602 1.800197    -----------------> Moment of Inertia 
            0.718277 1.800197 29.261652)*10^-6 [kg.m^2]"""

    def __init__(self, dt, id, state, uav_params, pload=False, lc=0):
        self.id        = id
        self.m         = float(uav_params['m'])
        self.I         = np.diag(uav_params['I'])
        self.invI      = linalg.inv(self.I)
        self.d         = float(uav_params['d']) 
        self.cft       = float(uav_params['cft'])
        self.maxThrust = float(uav_params['mxTh']) # [g] per motor
        arm           = 0.707106781*self.d
        self.invAll = np.array([
            [0.25, -(0.25 / arm), -(0.25 / arm), -(0.25 / self.cft)],
            [0.25, -(0.25 / arm),  (0.25 / arm),  (0.25 / self.cft)],
            [0.25,  (0.25 / arm),  (0.25 / arm), -(0.25 / self.cft)],
            [0.25,  (0.25 / arm), -(0.25 / arm),  (0.25 / self.cft)]
        ])     
        self.ctrlAll   = linalg.inv(self.invAll)
        self.grav     = np.array([0,0,-self.m*9.81])
        self.pload    = pload # default is false (no payload)
        self.lc       = lc # default length of cable is zero (no payload)
            ### State initialized with the Initial values ###
            ### state = [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, wx, wy, wz]
        self.state = state
        self.dt    = dt
        self.a     = np.zeros(3,)
        self.controller = uav_params['controller']
        self.ctrlPayload = 0
        self.hyperrpy = np.zeros(3,)
        self.hyperyaw = 0 
        self.fullState = np.empty((1,16))
        self.ctrlInps  = np.empty((1,8))
        self.refState  = np.empty((1,12))
        self.actions  = np.empty((1,4))
        self.u_all = np.empty((1,3))
        self.drag  = float((uav_params['drag']))
        if self.drag ==  1:
            self.Kaero = np.diag([-9.1785e-7, -9.1785e-7, -10.311e-7]) 
        if self.pload: 
            # for collision avoidance
            self.hpStack   = {}
            self.hp_prev = {}
            self.hpNums  = uav_params['NumOfHplane']
            self.radius  = uav_params['radius']
            for hpIds in range(self.hpNums):
                self.hpStack[hpIds] = np.empty((1,4))
                self.hp_prev[hpIds] = np.empty((1,4))

    def __str__(self):
        return "\nUAV object with physical parameters defined as follows: \n \n m = {} kg, l_arm = {} m \n \n{} {}\n I = {}{} [kg.m^2] \n {}{}\n\n Initial State = {}".format(self.m,self.d,'     ',self.I[0,:],' ',self.I[1,:],'     ',self.I[2,:], self.state)
        
    def getNextAngularState(self, curr_w, curr_q, tau):
        wdot  = self.invI @ (tau - skew(curr_w) @ self.I @ curr_w)
        wNext = wdot * self.dt + curr_w
        qNext = self.integrate_quat(curr_q, curr_w, self.dt)
        return qNext, wNext
        
    def integrate_quat(self, q, wb, dt):
        return multiply(q, exp(_promote_vec(wb * dt / 2))) 

    def getNextLinearState(self, curr_vel, curr_position, q ,fz, fa):
        R_IB = to_matrix(q)
        self.a =  (1/self.m) * (self.grav + R_IB @ np.array([0,0,fz]) + fa)
        velNext = self.a * self.dt + curr_vel
        posNext = curr_vel * self.dt + curr_position
        return posNext, velNext

    def states_evolution(self, control_t):
        """this method generates the 6D states evolution for the UAV given for each time step:
            the control input: f_th = [f1, f2, f3, f4] for the current step"""
        f_motors, control_t = self.computeFmotors(control_t) 
        w_motors            = self.wMotors(f_motors) #rotors angular velocities [rad/s]

        if self.drag == 1:
            fa = self.simpleDragModel(w_motors) # Simple Aerodynamic Drag Model
        else: 
            fa = np.zeros((3,))

        fz    = control_t[0]
        tau_i = control_t[1::]

        curr_pos  = self.state[0:3]  # position: x,y,z
        curr_vel  = self.state[3:6]  # linear velocity: xdot, ydot, zdot
        curr_q    = self.state[6:10] # quaternions: [qw, qx, qy, qz]
        curr_w    = self.state[10::]  # angular velocity: wx, wy, wz
        
        posNext, velNext = self.getNextLinearState(curr_vel, curr_pos, curr_q, fz, fa)
        qNext, wNext     = self.getNextAngularState(curr_w, curr_q, tau_i)
        
        self.state[0:3]  = posNext  # position: x,y,z
        self.state[3:6]  = velNext  # linear velocity: xdot, ydot, zdot
        self.state[6:10] = qNext# quaternions: [qw, qx, qy, qz]
        self.state[10::] = wNext # angular velocity: wx, wy, wz
    
        return self.state

    def computeFmotors(self, control_t):
        thrust = control_t[0]
        torque = control_t[1::]
        thrustpart = 0.25*thrust # N per rotor
        yawpart    = -0.25*torque[2] / self.cft

        arm        = 0.707106781*self.d
        rollpart   = (0.25 / arm) * torque[0]
        pitchpart  = (0.25 / arm) * torque[1]

        motorForce = np.zeros(4,)

        motorForce[0] = thrustpart - rollpart - pitchpart + yawpart
        motorForce[1] = thrustpart - rollpart + pitchpart - yawpart
        motorForce[2] = thrustpart + rollpart + pitchpart + yawpart
        motorForce[3] = thrustpart + rollpart - pitchpart - yawpart
        
        motorForceG = (motorForce/9.81)*1000
        motorForceG_clipped = np.clip(motorForceG, 0, self.maxThrust)

        motorForce = motorForceG_clipped*9.81/1000
        mu, sigma = 0, 0
        noise = np.random.normal(mu,sigma, 4)
        noise = np.zeros(4,)
        motorForce += noise
        return motorForce, self.ctrlAll @ motorForce
    
    def stackStandCtrl(self, state, control_t, u_all, ref_state):
        ## This method stacks the actual and reference states of the UAV 
        ## and the control input vector [fz taux, tauy, tauz, f1, f2, f3, f4]
        curr_w = self.state[10::]
        wd    = self.invI @ (control_t[1::] - skew(curr_w) @ self.I @ curr_w)
        state = np.hstack((state,wd))
        self.fullState  = np.vstack((self.fullState, state))

        f_motors   = self.invAll @ control_t
        f_motors_ = f_motors.copy()
        f_motorsG  =  (f_motors/9.81)*1000
        f_motorsG_clipped   = np.clip(f_motorsG, 0, self.maxThrust)
        f_motors = f_motorsG_clipped*9.81/1000
        self.ctrlInps   = np.vstack((self.ctrlInps, np.array([control_t, f_motors]).reshape(1,8)))
        self.actions    = np.vstack((self.actions, np.array(f_motors_).reshape(1,4)))
        self.u_all      = np.vstack((self.u_all, np.array(u_all).reshape(1,3)))
        self.refState   = np.vstack((self.refState, ref_state))
    
    def cursorUpwPl(self):
        self.fullState = np.delete(self.fullState, 0, 0)
        self.ctrlInps  = np.delete(self.ctrlInps,  0, 0)

    def cursorUp(self):
        ## This method removes the first row of the stack which is initialized as an empty array
        self.fullState = np.delete(self.fullState, 0, 0)
        self.ctrlInps  = np.delete(self.ctrlInps,  0, 0)
        self.refState  = np.delete(self.refState,  0, 0)
        self.actions  = np.delete(self.actions,   0, 0)
        self.u_all  = np.delete(self.u_all,   0, 0)

    def wMotors(self, f_motor):
        """This method transforms the current thrust for each motor to command input to angular velocity  in [rad/s]"""
        coef    = [5.484560e-4, 1.032633e-6 , 2.130295e-11]
        w_motors = np.empty((4,))
        cmd = 0
        for i in range(0,len(f_motor)):
            coef[0] = coef[0] - f_motor[i]
            poly_   = poly(coef) 
            roots_  = poly_.roots()
            for j in range(0, 2):
                if (roots_[j] >= 0):
                    cmd = roots_[j]
            w_motors[i] = 0.04076521*cmd + 380.8359
        return w_motors    
    
    def simpleDragModel(self, w_motors):
        wSum = np.sum(w_motors)
        R_IB = to_matrix(self.state[6:10])
        fa   = wSum * self.Kaero @ np.transpose(R_IB) @ self.state[3:6]
        return fa

    def addHp(self, hpId, hp):
        coeffs = hp.coeffs()
        self.hpStack[hpId]  =  np.vstack((self.hpStack[hpId], coeffs))
        self.hp_prev[hpId]  =  coeffs

    def removeEmptyRow(self):
        for hpsIds in self.hpStack.keys():
            self.hpStack[hpsIds] = np.delete(self.hpStack[hpsIds], 0,0)
