import cfusdlog
import matplotlib.pyplot as plt
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from mpl_toolkits import mplot3d 
import matplotlib.animation as animation
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.gridspec import SubplotSpec, GridSpec

def create_subtitle(fig: plt.Figure, grid: SubplotSpec, title: str):
    row = fig.add_subplot(grid)
    row.set_title('\n\n\n'+title, fontweight='medium',fontsize='medium')
    row.set_frame_on(False)
    row.axis('off')


def setlimits(ax, full_state):
    # This method finds the maximum value in the x-y-z actual states and sets the limits of the figure accordingly   
    # edge: adds extra space for the figure 
    edge  = 0.4
    max_x = max(full_state[:,0])
    max_y = max(full_state[:,1])
    max_z = max(full_state[:,2])
    if (max_x >= max_y) and (max_x >= max_z):
        max_ = max_x
        ax.set_xlim3d([-max_-edge, max_+edge])
        ax.set_ylim3d([-max_-edge, max_+edge])
        ax.set_zlim3d([-max_-edge, max_+edge])
    elif (max_y >= max_x) and (max_y >= max_z):
        max_ = max_y
        ax.set_xlim3d([-max_-edge, max_+edge])
        ax.set_ylim3d([-max_-edge, max_+edge])
        ax.set_zlim3d([-max_-edge, max_+edge])
    else:
        max_ = max_z
        ax.set_xlim3d([-max_-edge, max_+edge])
        ax.set_ylim3d([-max_-edge, max_+edge])
        ax.set_zlim3d([-max_-edge, max_+edge])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    return ax

def find_nearest(a, a0):
    "Element in nd array `a` closest to the scalar value `a0`"
    idx = np.abs(a - a0).argmin()
    return idx, a.flat[idx]

def preparefigs(powerDist, controller):
    fig1, ax1 = plt.subplots(3, 1, sharex=True)
    fig1.tight_layout()
    
    fig2, ax2 = plt.subplots(3, 1, sharex=True)
    fig2.tight_layout()

    fig3, ax3 = plt.subplots(3, 1, sharex=True)
    fig3.tight_layout()

    fig4, ax4 = plt.subplots(3, 1, sharex=True)
    fig4.tight_layout()

    fig5, ax5 = plt.subplots(2, 3, sharex=True)
    fig5.tight_layout()

    fig6 = plt.figure(constrained_layout=True)
    gs = GridSpec(3, 2, figure=fig6)
    ax6 = fig6.add_subplot(gs[:, 0])
    ax7 = fig6.add_subplot(gs[0,1])
    ax8 = fig6.add_subplot(gs[1,1],sharey=ax7)
    ax9 = fig6.add_subplot(gs[2,1],sharey=ax7)
    
    fig7 = plt.figure(figsize=(10,10))
    ax10 = fig7.add_subplot(autoscale_on=True,projection="3d")
    if powerDist:
        fig8, ax11 =  plt.subplots(5, 1)
        fig8.tight_layout()
    elif not powerDist and controller == 'leep':
        fig8, ax11 =  plt.subplots(3, 1)
        fig8.tight_layout()
        return fig1, ax1, fig2, ax2, fig3, ax3, fig4, ax4, fig5, ax5, fig6, gs, ax6, ax7, ax8, ax9, fig7, ax10, fig8, ax11
    elif not powerDist and controller == 'lee':
        return fig1, ax1, fig2, ax2, fig3, ax3, fig4, ax4, fig5, ax5, fig6, gs, ax6, ax7, ax8, ax9, fig7, ax10
    elif not powerDist and controller == 'leep':
        return fig1, ax1, fig2, ax2, fig3, ax3, fig4, ax4, fig5, ax5, fig6, gs, ax6, ax7, ax8, ax9, fig7, ax10, fig8, ax11

def main(args):
    # decode binary log data
    maxtime = args.maxtime
    logData = cfusdlog.decode(args.file_usd)['fixedFrequency']
    # print(logData.keys())
    # exit()
    time = np.column_stack(logData['timestamp']/1000).flatten()
    time = time - time[0]

    if maxtime > time[-1]:
        maxtime = time[-1]
        time = time[0::]
    else:
        idx, value = find_nearest(time, maxtime)
        time = time[0:idx+1]
        for id in logData.keys():
            logData[id] = logData[id][0:idx+1]

    logDataKeys = list(logData.keys())
    controller = args.controller

    if controller == 'sjc':
        ctrl = 'ctrlSJC.'
    elif controller == 'lee':
        ctrl = 'ctrlLee.'
        Kpxy = logData['ctrlLee.Kpos_Px'][0]
        Kpz  = logData['ctrlLee.Kpos_Pz'][0]
        Kdxy = logData['ctrlLee.Kpos_Dx'][0]
        Kdz  = logData['ctrlLee.Kpos_Dz'][0]

        Krxy = logData['ctrlLee.KR_x'][0]
        Krz  = logData['ctrlLee.KR_z'][0]
        Kwxy = logData['ctrlLee.Kw_x'][0]
        Kwz  = logData['ctrlLee.Kw_z'][0]             
    elif controller == 'leep':
        ctrl = 'ctrlLeeP.' 
        Kpxy = logData['ctrlLeeP.Kpos_Px'][0]
        Kpz  = logData['ctrlLeeP.Kpos_Pz'][0]
        Kdxy = logData['ctrlLeeP.Kpos_Dx'][0]
        Kdz  = logData['ctrlLeeP.Kpos_Dz'][0]

        Kqxy = logData['ctrlLeeP.Kqx'][0]
        Kqz  = logData['ctrlLeeP.Kqz'][0]
        Kwxy = logData['ctrlLeeP.Kwx'][0]
        Kwz  = logData['ctrlLeeP.Kwz'][0]


    filename = args.filename
    print('Plotting...')
    f = PdfPages(filename+'.pdf')
    
    plt.rcParams['axes.grid'] = True
    plt.rcParams['figure.max_open_warning'] = 100

    posq = np.column_stack((
    logData['stateEstimateZ.x'] / 1000.0,
    logData['stateEstimateZ.y'] / 1000.0,
    logData['stateEstimateZ.z'] / 1000.0))

    linVel = np.column_stack((
    logData['stateEstimateZ.vx'] / 1000.0,
    logData['stateEstimateZ.vy'] / 1000.0,
    logData['stateEstimateZ.vz'] / 1000.0))
    if controller == 'leep':
        posp = np.column_stack((
        logData['stateEstimateZ.px'] / 1000.0,
        logData['stateEstimateZ.py'] / 1000.0,
        logData['stateEstimateZ.pz'] / 1000.0))

        linVelp = np.column_stack((
        logData['stateEstimateZ.pvx'] / 1000.0,
        logData['stateEstimateZ.pvy'] / 1000.0,
        logData['stateEstimateZ.pvz'] / 1000.0))
        
        ui = np.column_stack((
        logData['ctrlLeeP.ux'],
        logData['ctrlLeeP.uy'],
        logData['ctrlLeeP.uz']))

        normq = (np.linalg.norm((posp-posq), axis=1)).reshape(len(posp),1)
        normq = np.repeat(normq, 3, axis=1)
        qi = np.divide((posp-posq), normq)
        alpha = 0.01
        linVelp_est = np.zeros_like(linVelp)
        qi_dot_prev = np.zeros(3,)
        linVelp_est[0,:] = linVel[0,:]
        for i in range(0, len(time)-1):
            qi_prev = qi[i,:]
            qi_curr = qi[i+1,:]
            dt = time[i+1] - time[i]
            qidot = (qi_curr - qi_prev)/dt 
            qidotfilt =  (1-alpha)*qi_dot_prev + alpha * qidot
            qi_dot_prev = qidotfilt
            linvp = linVel[i,:] + 0.663*qidotfilt
            linVelp_est[i+1,:] = linvp
 
    posdes = np.column_stack((
    logData['ctrltargetZ.x'] / 1000.0,
    logData['ctrltargetZ.y'] / 1000.0,
    logData['ctrltargetZ.z'] / 1000.0))

    linVeldes = np.column_stack((
    logData['ctrltargetZ.vx'] / 1000.0,
    logData['ctrltargetZ.vy'] / 1000.0,
    logData['ctrltargetZ.vz'] / 1000.0))

    angVel = np.column_stack((
    logData[ctrl+'omegax'],
    logData[ctrl+'omegay'],
    logData[ctrl+'omegaz']))

    angVeldes = np.column_stack((
    logData[ctrl+'omegarx'],
    logData[ctrl+'omegary'],
    logData[ctrl+'omegarz']))

    cont_stack = np.column_stack((
    logData[ctrl+'thrustSI'],
    logData[ctrl+'torquex'] ,
    logData[ctrl+'torquey'] ,
    logData[ctrl+'torquez'] ))
 
    if controller == 'lee' or controller == 'leep':
        rpydes = np.column_stack((
        logData[ctrl+'rpydx'],
        logData[ctrl+'rpydy'],
        logData[ctrl+'rpydz']))
        
        rpy = np.column_stack((
        logData[ctrl+'rpyx'],
        logData[ctrl+'rpyy'],
        logData[ctrl+'rpyz']))
        
    elif controller == 'sjc':
        rpydes = np.column_stack((
        logData['ctrlSJC.qx'],
        logData['ctrlSJC.qy'],
        logData['ctrlSJC.qz']))

        rpy = np.column_stack((
        logData['ctrlSJC.qrx'],
        logData['ctrlSJC.qry'],
        logData['ctrlSJC.qrz']))

    powerDist = False
    if 'powerDist.' in  (' ').join(logData.keys()):
        powerDist = True
        thrust = np.column_stack((
        logData['powerDist.thrustPart']/(100)
        )).reshape(time.shape) 

        roll   = np.column_stack((
        logData['powerDist.rollPart']/(100)
        )).reshape(time.shape) 

        pitch  = np.column_stack((
        logData['powerDist.pitchPart']/(100)
        )).reshape(time.shape) 

        yaw    = np.column_stack((
        logData['powerDist.yawPart']/(100)
        )).reshape(time.shape) 

        maxTh  = np.column_stack((
        logData['powerDist.maxThrust']/(100)
        )).reshape(time.shape) 

        motorForces = np.row_stack(( 
        thrust - roll - pitch + yaw,
        thrust - roll + pitch - yaw,
        thrust + roll + pitch + yaw,
        thrust + roll - pitch - yaw
        )).T

    if controller == 'lee' or controller == 'sjc':
        poserr = (posdes[:,:] - posq[:,:]).reshape(len(posq),3)
        linVerr = (linVeldes[:,:] - linVel[:,:]).reshape(len(posq),3) 
    elif controller == 'leep':
        poserr = (posdes[:,:] - posp[:,:]).reshape(len(posp),3)
        linVerr = (linVeldes[:,:] - linVel[:,:]).reshape(len(posp),3)
    ts = 'time [s]'
    if powerDist:
        fig1, ax1, fig2, ax2, fig3, ax3, fig4, ax4, fig5, ax5, \
        fig6, gs, ax6, ax7, ax8, ax9, fig7, ax10, fig8, ax11\
        = preparefigs(powerDist, controller)
    elif not powerDist and controller == 'lee':
        fig1, ax1, fig2, ax2, fig3, ax3, fig4, ax4, fig5, ax5, \
        fig6, gs, ax6, ax7, ax8, ax9, fig7, ax10\
        = preparefigs(powerDist, controller)
    elif not powerDist and controller == 'leep':
        fig1, ax1, fig2, ax2, fig3, ax3, fig4, ax4, fig5, ax5, \
        fig6, gs, ax6, ax7, ax8, ax9, fig7, ax10, fig8, ax11\
        = preparefigs(powerDist, controller)


    ax1[0].plot(time, posq[:,0], c='k', lw=0.75,label='Actual of UAV'), ax1[1].plot(time, posq[:,1], lw=0.75, c='k'), ax1[2].plot(time, posq[:,2], lw=0.75, c='k')
    if controller == 'leep':
        ax1[0].plot(time, posp[:,0], c='b', lw=0.75,label='Actual of load'), ax1[1].plot(time, posp[:,1], lw=0.75, c='b'), ax1[2].plot(time, posp[:,2], lw=0.75, c='b')
    ax1[0].plot(time, posdes[:,0], lw=0.75, c='darkgreen',label='Reference'), ax1[1].plot(time, posdes[:,1], lw=0.75, c='darkgreen'), ax1[2].plot(time, posdes[:,2], lw=0.75, c='darkgreen')
    ax1[0].set_ylabel('x [m]',), ax1[1].set_ylabel('y [m]'), ax1[2].set_ylabel('z [m]')
    ax1[0].legend()
    fig1.supxlabel(ts,fontsize='small')

    grid = plt.GridSpec(3,1)
    create_subtitle(fig1, grid[0, ::], 'Actual vs Reference Positions')

    ###################################
    
    ax2[0].plot(time, linVel[:,0],lw=0.75, c='k' ,label='Actual UAV'), ax2[1].plot(time, linVel[:,1],lw=0.75, c='k'), ax2[2].plot(time, linVel[:,2],lw=0.75, c='k')
    if controller == 'leep':
        ax2[0].plot(time, linVelp[:,0], c='b', lw=0.75,label='Actual filt vel'), ax2[1].plot(time, linVelp[:,1], lw=0.75, c='b'), ax2[2].plot(time, linVelp[:,2], lw=0.75, c='b')
        # ax2[0].plot(time, linVelp_est[:,0], c='m', lw=0.75,label='Actual filt q'), ax2[1].plot(time, linVelp_est[:,1], lw=.75, c='m'), ax2[2].plot(time, linVelp_est[:,2], lw=.75, c='m')

    ax2[0].plot(time, linVeldes[:,0],lw=0.75, c='darkgreen',label='Reference'), ax2[1].plot(time, linVeldes[:,1],lw=0.75, c='darkgreen'), ax2[2].plot(time, linVeldes[:,2],lw=0.75, c='darkgreen')
    ax2[0].set_ylabel('vx [m/s]'), ax2[1].set_ylabel('vy [m/s]'), ax2[2].set_ylabel('vz [m/s]')
    ax2[0].legend()
    fig2.supxlabel(ts,fontsize='small')

    grid = plt.GridSpec(3,1)
    create_subtitle(fig2, grid[0, ::], 'Actual vs Reference Linear Velocities')

    ###################################
    ax3[0].plot(time, np.degrees(rpy[:,0]), c='k',lw=0.5,label='Actual')
    ax3[1].plot(time, np.degrees(rpy[:,1]), c='k',lw=0.5,label='Actual')
    ax3[2].plot(time, np.degrees(rpy[:,2]), c='k',lw=0.5,label='Actual')
    ax3[0].plot(time, np.degrees(rpydes[:,0]) ,lw=0.5, c='darkgreen',label='Reference')
    ax3[1].plot(time, np.degrees(rpydes[:,1]) ,lw=0.5, c='darkgreen',label='Reference')
    ax3[2].plot(time, np.degrees(rpydes[:,2]) ,lw=0.5, c='darkgreen',label='Reference')

    ax3[0].set_ylabel('r [deg]',labelpad=-2), ax3[1].set_ylabel('p [deg]',labelpad=-2), ax3[2].set_ylabel('y [deg]',labelpad=-2)
    fig3.supxlabel(ts,fontsize='small')

    grid = plt.GridSpec(3,1)
    create_subtitle(fig3, grid[0, ::], 'Actual vs Reference Angular Velocities')

    ###################################
    ax4[0].plot(time, np.degrees(angVel[:,0]), c='k',lw=0.5,label='Actual')
    ax4[1].plot(time, np.degrees(angVel[:,1]), c='k',lw=0.5,label='Actual')
    ax4[2].plot(time, np.degrees(angVel[:,2]), c='k',lw=0.5,label='Actual')
    ax4[0].plot(time, np.degrees(angVeldes[:,0]), lw=0.5, c='darkgreen',label='Reference')
    ax4[1].plot(time, np.degrees(angVeldes[:,1]), lw=0.5, c='darkgreen',label='Reference')
    ax4[2].plot(time, np.degrees(angVeldes[:,2]), lw=0.5, c='darkgreen',label='Reference')

    ax4[0].set_ylabel('wx [deg/s]',labelpad=-2), ax4[1].set_ylabel('wy [deg/s]',labelpad=-2), ax4[2].set_ylabel('wz [deg/s]',labelpad=-2)
    fig4.supxlabel(ts,fontsize='small')
    max_x = abs(max(np.degrees(angVel[:,0]),key=abs))
    max_y = abs(max(np.degrees(angVel[:,1]),key=abs))
    max_z = abs(max(np.degrees(angVel[:,2]),key=abs))
    # ax4[0].set_ylim(-max_x, max_x), ax4[1].set_ylim(-max_y, max_y), ax4[2].set_ylim(-max_z, max_z)
   
    grid = plt.GridSpec(3,1)
    create_subtitle(fig3, grid[0, ::], 'Actual vs Reference Angular Velocities')

    ###################################

    ax5[0,0].plot(time, poserr[:,0],c='r',lw=0.5), ax5[0,1].plot(time, poserr[:,1],c='r',lw=0.5), ax5[0,2].plot(time, poserr[:,2],c='r',lw=0.5)
    ax5[0,0].set_ylabel('ex [m]'), ax5[0,1].set_ylabel('ey [m]'), ax5[0,2].set_ylabel('ez [m]')
    
    ax5[1,0].plot(time, linVerr[:,0],c='r',lw=0.5), ax5[1,1].plot(time, linVerr[:,1],c='r',lw=0.5), ax5[1,2].plot(time, linVerr[:,2],c='r',lw=0.5)
    ax5[1,0].set_ylabel('vex [m/s]'), ax5[1,1].set_ylabel('vey [m/s]'), ax5[1,2].set_ylabel('vez [m/s]')
    fig5.supxlabel(ts,fontsize='small')

    grid = plt.GridSpec(2,3)
    create_subtitle(fig5, grid[0, ::], 'Positional errors')
    create_subtitle(fig5, grid[1, ::], 'Linear Velocities errors')
    
    ###################################

    ax6.plot(time, cont_stack[:,0],lw=0.5, c='darkblue')
    ax6.set_ylabel('fz [N]')
    
    ax7.plot(time, cont_stack[:,1], lw=0.5, c='darkblue'), ax8.plot(time, cont_stack[:,2], lw=0.5, c='darkblue'), ax9.plot(time, cont_stack[:,3],lw=0.5, c='darkblue')
    ax7.set_ylabel('taux [N.m]',fontsize='small'), ax8.set_ylabel('tauy [N.m]',fontsize='small'), ax9.set_ylabel('tauz [N.m]',fontsize='small')
    fig6.supxlabel(ts,fontsize='small')

    create_subtitle(fig6, gs[::, 0], 'Force Control Input')
    create_subtitle(fig6, gs[::, 1], 'Torque Control Input')

    ###################################
    if controller == 'leep':
        ax10.plot3D(posp[:,0], posp[:,1], posp[:,2], 'r-.',lw=1, label="Actual Trajectory Load")
    ax10.plot3D(posq[:,0], posq[:,1], posq[:,2], 'k-.',lw=1, label="Actual Trajectory UAV")
    ax10.plot3D(posdes[:,0], posdes[:,1] , posdes[:,2],'darkgreen',ls='--',lw=1,label="Reference Trajectory")
    ax10.legend()
    ax10 = setlimits(ax10, posq)
   ################################### 
    if powerDist:
        ax11[0].plot(time,thrust, c='k',lw=0.5), 
        ax11[0].set_ylabel('thrust [g]')        
        ax11[1].plot(time, pitch, c='k',lw=0.5)
        ax11[1].set_ylabel('pitch [g]')        
        ax11[2].plot(time, roll, c='k',lw=0.5)
        ax11[2].set_ylabel('roll [g]')        
        ax11[3].plot(time, yaw, c='k',lw=0.5)
        ax11[3].set_ylabel('yaw [g]')        

        ax11[4].plot(time, motorForces[:,0],c='m',lw=0.5, label='m1')
        ax11[4].plot(time, motorForces[:,1],c='b',lw=0.5, label='m2')
        ax11[4].plot(time, motorForces[:,2],c='g',lw=0.5, label='m3')
        ax11[4].plot(time, motorForces[:,3],c='k',lw=0.5, label='m4')
        ax11[4].plot(time, maxTh, c='r', lw=0.9, label='MAX_T')
        # print(maxTh[-1])
        ax11[4].set_ylabel('F [g]')        

        fig8.supxlabel(ts,fontsize='small')

        grid = plt.GridSpec(5,1)
        create_subtitle(fig8, grid[0, ::], 'Thrust')
        create_subtitle(fig8, grid[1, ::], 'Pitch')
        create_subtitle(fig8, grid[2, ::], 'roll')
        create_subtitle(fig8, grid[3, ::], 'Yaw')
        create_subtitle(fig8, grid[4, ::], 'Motor Forces')
    
    elif not powerDist and controller == 'leep':
        ax11[0].plot(time, ui[:,0], c='k',lw=0.5,label='desired Inp x')
        ax11[1].plot(time, ui[:,1], c='k',lw=0.5,label='desired Inp y')
        ax11[2].plot(time, ui[:,2], c='k',lw=0.5,label='desired Inp z')
        ax11[0].set_ylabel('ux ',labelpad=-2), ax11[1].set_ylabel('uy ',labelpad=-2), ax11[2].set_ylabel('uz ',labelpad=-2)
        fig8.supxlabel(ts,fontsize='small')
        grid = plt.GridSpec(3,1)
        create_subtitle(fig8, grid[0, ::], 'Desired Forces on Payload')

   ###################################
    if controller == 'lee':
        txt = 'Kp = [{:.2f},{:.2f},{:.2f}]\n \
        Kd = [{:.2f},{:.2f},{:.2f}]\n \
        KR = [{:.4f},{:.4f},{:.4f}]\n \
        Kw = [{:.4f},{:.4f},{:.4f}]'.format(Kpxy, Kpxy, Kpz, Kdxy, Kdxy, Kdz, Krxy, Krxy, Krz, Kwxy, Kwxy, Kwz)
        textfig, textax = plt.subplots(figsize=(3, 3))
        textax.grid(False)
        textax.axis(False)
        textax.text(0, 1, txt, size=13, color='black')
        textfig.savefig(f, format='pdf', bbox_inches='tight')
    elif controller =='leep':
        txt = 'Kp = [{:.2f},{:.2f},{:.2f}]\n \
        Kd = [{:.2f},{:.2f},{:.2f}]\n \
        Kq = [{:.4f},{:.4f},{:.4f}]\n \
        Kw = [{:.4f},{:.4f},{:.4f}]'.format(Kpxy, Kpxy, Kpz, Kdxy, Kdxy, Kdz, Kqxy, Kqxy, Kqz, Kwxy, Kwxy, Kwz)
        textfig, textax = plt.subplots(figsize=(3, 3))
        textax.grid(False)
        textax.axis(False)
        textax.text(0, 1, txt, size=13, color='black')
        textfig.savefig(f, format='pdf', bbox_inches='tight')

    fig1.savefig(f, format='pdf', bbox_inches='tight')
    fig2.savefig(f, format='pdf', bbox_inches='tight')
    fig3.savefig(f, format='pdf', bbox_inches='tight')        
    fig4.savefig(f, format='pdf', bbox_inches='tight')  
    fig5.savefig(f, format='pdf', bbox_inches='tight')  
    fig6.savefig(f, format='pdf', bbox_inches='tight')  
    if powerDist or controller == 'leep':
        fig8.savefig(f, format='pdf', bbox_inches='tight')      
    fig7.savefig(f, format='pdf', bbox_inches='tight')
    f.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_usd")
    parser.add_argument("controller")
    parser.add_argument("filename")
    parser.add_argument("--maxtime",type=float, default=10000)
    args = parser.parse_args()
    main(args)
