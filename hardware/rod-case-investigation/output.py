import cfusdlog
import numpy as np
import rowan as rn
import yaml
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from mpl_toolkits import mplot3d 
import matplotlib.animation as animation
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.gridspec import SubplotSpec, GridSpec
import matplotlib.pyplot as plt
plt.rcParams['axes.grid'] = True
plt.rcParams['figure.max_open_warning'] = 100

def create_subtitle(fig: plt.Figure, grid: SubplotSpec, title: str):
    row = fig.add_subplot(grid)
    row.set_title('\n\n\n'+title, fontweight='medium',fontsize='medium')
    row.set_frame_on(False)
    row.axis('off')


def plotload(states ,dstates, time):
    fig1, ax1 = plt.subplots(3, 1, sharex=True)
    fig1.tight_layout()
    posp = states[:,0:3]
    posdes = dstates[:,0:3]
    ts = 'time [s]'

    ax1[0].plot(time, posp[:,0], c='b', lw=0.75,label='Actual of load'), ax1[1].plot(time, posp[:,1], lw=0.75, c='b'), ax1[2].plot(time, posp[:,2], lw=0.75, c='b')
    ax1[0].plot(time, posdes[:,0], lw=0.75, c='darkgreen',label='Reference'), ax1[1].plot(time, posdes[:,1], lw=0.75, c='darkgreen'), ax1[2].plot(time, posdes[:,2], lw=0.75, c='darkgreen')
    ax1[0].set_ylabel('x [m]',), ax1[1].set_ylabel('y [m]'), ax1[2].set_ylabel('z [m]')
    ax1[0].legend()
    fig1.supxlabel(ts,fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(fig1, grid[0, ::], 'Positions of the load')

    fig2, ax2 = plt.subplots(3, 1, sharex=True)
    fig2.tight_layout()
    linVelp = states[:,3:6]
    linVeldes = dstates[:,3:6]
    ax2[0].plot(time, linVelp[:,0], c='b', lw=0.75,label='Actual filt vel'), ax2[1].plot(time, linVelp[:,1], lw=0.75, c='b'), ax2[2].plot(time, linVelp[:,2], lw=0.75, c='b')
    ax2[0].plot(time, linVeldes[:,0],lw=0.75, c='darkgreen',label='Reference'), ax2[1].plot(time, linVeldes[:,1],lw=0.75, c='darkgreen'), ax2[2].plot(time, linVeldes[:,2],lw=0.75, c='darkgreen')
    ax2[0].set_ylabel('vx [m/s]'), ax2[1].set_ylabel('vy [m/s]'), ax2[2].set_ylabel('vz [m/s]')
    ax2[0].legend()
    fig2.supxlabel(ts,fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(fig2, grid[0, ::], 'Linear Velocities of the load')

    if not np.isnan((states[:,6:10])).any():
        fig3, ax3 = plt.subplots(3, 1, sharex=True)
        fig3.tight_layout()
        rpy = rn.to_euler(states[:,6:10], convention='xyz')
        rpydes = rn.to_euler(dstates[:,6:10], convention='xyz')
        ax3[0].plot(time, np.degrees(rpy[:,0]), c='b',lw=0.5,label='Actual')
        ax3[0].plot(time, np.degrees(rpydes[:,0]), c='darkgreen',lw=0.5,label='Ref')
        ax3[1].plot(time, np.degrees(rpy[:,1]), c='b',lw=0.5,label='Actual')
        ax3[1].plot(time, np.degrees(rpydes[:,1]), c='darkgreen',lw=0.5,label='Ref')
        ax3[2].plot(time, np.degrees(rpy[:,2]), c='b',lw=0.5,label='Actual')
        ax3[2].plot(time, np.degrees(rpydes[:,2]), c='darkgreen',lw=0.5,label='Ref')

        ax3[0].set_ylabel('r [deg]',labelpad=-2), ax3[1].set_ylabel('p [deg]',labelpad=-2), ax3[2].set_ylabel('y [deg]',labelpad=-2)
        fig3.supxlabel(ts,fontsize='small')

        grid = plt.GridSpec(3,1)
        create_subtitle(fig3, grid[0, ::], 'rpy of the load')
        
        fig4, ax4 = plt.subplots(3, 1, sharex=True)
        fig4.tight_layout()
        omega = states[:,10:13]
        # omegar = dstates[:,10:13]

        ax4[0].plot(time, np.degrees(omega[:,0]), c='b',lw=0.5,label='Actual')
        ax4[1].plot(time, np.degrees(omega[:,1]), c='b',lw=0.5,label='Actual')
        ax4[2].plot(time, np.degrees(omega[:,2]), c='b',lw=0.5,label='Actual')

        # ax4[0].plot(time, np.degrees(omegar[:,0]), c='k',lw=0.5,label='Ref')
        # ax4[1].plot(time, np.degrees(omegar[:,1]), c='k',lw=0.5,label='Ref')
        # ax4[2].plot(time, np.degrees(omegar[:,2]), c='k',lw=0.5,label='Ref')

        ax4[0].set_ylabel('wx [deg/s]',labelpad=-2), ax4[1].set_ylabel('wy [deg/s]',labelpad=-2), ax4[2].set_ylabel('wz [deg/s]',labelpad=-2)
        fig4.supxlabel(ts,fontsize='small')

        grid = plt.GridSpec(3,1)
        create_subtitle(fig4, grid[0, ::], 'omega of the load')
        return fig1, fig2, fig3, fig4
    else:
        return fig1, fig2

def plotuav(states, dstates, time ,name):
    fig1, ax1 = plt.subplots(3, 1, sharex=True)
    fig1.tight_layout()
    posp = states[:,0:3]
    posdes = dstates[:,0:3]
    ts = 'time [s]'

    ax1[0].plot(time, posp[:,0], c='b', lw=0.75,label='Actual of ' + name), ax1[1].plot(time, posp[:,1], lw=0.75, c='b'), ax1[2].plot(time, posp[:,2], lw=0.75, c='b')
    ax1[0].set_ylabel('x [m]',), ax1[1].set_ylabel('y [m]'), ax1[2].set_ylabel('z [m]')
    ax1[0].legend()
    fig1.supxlabel(ts,fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(fig1, grid[0, ::], 'UAV Positions of '+ name)
    
    v = states[:,3:6]
    figv, axv = plt.subplots(3, 1, sharex=True)
    figv.tight_layout()
    axv[0].plot(time, v[:,0], c='b', lw=0.75,label='Actual of ' + name), axv[1].plot(time, v[:,1], lw=0.75, c='b'), axv[2].plot(time, v[:,2], lw=0.75, c='b')
    axv[0].set_ylabel('vx [m/s]',), axv[1].set_ylabel('vy [m/s]'), axv[2].set_ylabel('vz [m/s]')
    axv[0].legend()
    figv.supxlabel(ts,fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(figv, grid[0, ::], 'UAV velocity of '+ name)
    
    
    fig2, ax2 = plt.subplots(3, 1, sharex=True)
    fig2.tight_layout()

    fig3, ax3 = plt.subplots(3, 1, sharex=True)
    fig3.tight_layout()
    rpy = rn.to_euler((states[:,6:10]), convention='xyz')
    rpydes = rn.to_euler(dstates[:,0:4], convention='xyz')
    ax2[0].plot(time, np.degrees(rpy[:,0]), c='b',lw=0.5,label='Actual')
    ax2[1].plot(time, np.degrees(rpy[:,1]), c='b',lw=0.5,label='Actual')
    ax2[2].plot(time, np.degrees(rpy[:,2]), c='b',lw=0.5,label='Actual')
    ax2[0].plot(time, np.degrees(rpydes[:,0]) ,lw=0.5, c='darkgreen',label='Reference')
    ax2[1].plot(time, np.degrees(rpydes[:,1]) ,lw=0.5, c='darkgreen',label='Reference')
    ax2[2].plot(time, np.degrees(rpydes[:,2]) ,lw=0.5, c='darkgreen',label='Reference')

    ax2[0].set_ylabel('r [deg]',labelpad=-2), ax3[1].set_ylabel('p [deg]',labelpad=-2), ax2[2].set_ylabel('y [deg]',labelpad=-2)
    fig2.supxlabel(ts,fontsize='small')

    grid = plt.GridSpec(3,1)
    create_subtitle(fig2, grid[0, ::], 'rpy of UAV: ' + name)

    angVel = states[:, 10::]
    angVeldes = dstates[:, 3::]
    ax3[0].plot(time, np.degrees(angVel[:,0]), c='b',lw=0.5,label='Actual')
    ax3[1].plot(time, np.degrees(angVel[:,1]), c='b',lw=0.5,label='Actual')
    ax3[2].plot(time, np.degrees(angVel[:,2]), c='b',lw=0.5,label='Actual')
    ax3[0].plot(time, np.degrees(angVeldes[:,0]), lw=0.5, c='darkgreen',label='Reference')
    ax3[1].plot(time, np.degrees(angVeldes[:,1]), lw=0.5, c='darkgreen',label='Reference')
    ax3[2].plot(time, np.degrees(angVeldes[:,2]), lw=0.5, c='darkgreen',label='Reference')

    ax3[0].set_ylabel('wx [deg/s]',labelpad=-2), ax3[1].set_ylabel('wy [deg/s]',labelpad=-2), ax3[2].set_ylabel('wz [deg/s]',labelpad=-2)
    fig3.supxlabel(ts,fontsize='small')
    max_x = abs(max(np.degrees(angVel[:,0]),key=abs))
    max_y = abs(max(np.degrees(angVel[:,1]),key=abs))
    max_z = abs(max(np.degrees(angVel[:,2]),key=abs))
    create_subtitle(fig3, grid[0, ::], 'Angular Velocities (omega) of '+ name)

    return fig1, fig2, fig3, figv

    pass

def plotcable(states, qd, time, cf):
    qi = states[:, 0:3]
    fig1, ax1 = plt.subplots(3, 1, sharex=True)
    fig1.tight_layout()
    ts = 'time [s]'
    
    ax1[0].plot(time, qi[:,0], c='b', lw=0.75,label='Actual'), ax1[1].plot(time, qi[:,1], lw=0.75, c='b'), ax1[2].plot(time, qi[:,2], lw=0.75, c='b')
    ax1[0].plot(time, qd[:,0], lw=0.75, c='darkgreen',label='Reference'), ax1[1].plot(time, qd[:,1], lw=0.75, c='darkgreen'), ax1[2].plot(time, qd[:,2], lw=0.75, c='darkgreen')
    ax1[0].set_ylabel('qix',), ax1[1].set_ylabel('qiy'), ax1[2].set_ylabel('qiz')
    ax1[0].legend()
    fig1.supxlabel(ts,fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(fig1, grid[0, ::], 'qi cable directions of '+ cf)
    fig2, ax2 = plt.subplots(3, 1, sharex=True)
    fig2.tight_layout()
    ax2[0].plot(time, states[:,3], c='b',lw=0.5,label='Actual')
    ax2[0].plot(time, states[:,6], c='darkgreen',lw=0.5,label='Ref')
    ax2[1].plot(time, states[:,4], c='b',lw=0.5,label='Actual')
    ax2[1].plot(time, states[:,7], c='darkgreen',lw=0.5,label='Ref')
    ax2[2].plot(time, states[:,5], c='b',lw=0.5,label='Actual')
    ax2[2].plot(time, states[:,8], c='darkgreen',lw=0.5,label='Ref')

    ax2[0].set_ylabel('qidotx',labelpad=-2), ax2[1].set_ylabel('qidoty',labelpad=-2), ax2[2].set_ylabel('qidotz',labelpad=-2)
    fig2.supxlabel(ts,fontsize='small')

    grid = plt.GridSpec(3,1)
    create_subtitle(fig2, grid[0, ::], 'qidot of ' + cf)
    return fig1, fig2


def main(args=None):
    
    # files = ["cf5_48", "cf6_48"]
    # att_points = [[0,-0.3,0], [0,0.3,0]]
    # shape = 'cuboid'

    files = ["cf5_99", "cf6_99"]
    att_points = [[0,-0.3,0], [0,0.3,0]]
    shape = 'point'

    logDatas = [cfusdlog.decode(f)['fixedFrequency'] for f in files]

    configData = {}
    configData['robots'] = {}
    configData['payload'] = 'payload.csv'
    if shape == 'point':
        configData['payload_type'] = 'point'
    elif shape == 'triangle':
        configData['payload_type'] = 'triangle'
    elif shape == 'rod':
        configData['payload_type'] = 'rod'
    elif shape == 'cuboid':
        configData['payload_type'] = 'cuboid'
    else:
        print('please add the right shape!')
        exit()

    f = PdfPages('results.pdf')


    # payload
    time1 = (logDatas[0]['timestamp']-logDatas[0]['timestamp'][0])/1000
    loadstates = np.array([ logDatas[0]['stateEstimateZ.px']/1000,
                            logDatas[0]['stateEstimateZ.py']/1000, 
                            logDatas[0]['stateEstimateZ.pz']/1000,
                            logDatas[0]['stateEstimateZ.pvx']/1000,
                            logDatas[0]['stateEstimateZ.pvy']/1000, 
                            logDatas[0]['stateEstimateZ.pvz']/1000, 
                            logDatas[0]['stateEstimate.pqw'],
                            logDatas[0]['stateEstimate.pqx'],
                            logDatas[0]['stateEstimate.pqy'],
                            logDatas[0]['stateEstimate.pqz'],
                            logDatas[0]['stateEstimate.pwx'],
                            logDatas[0]['stateEstimate.pwy'],
                            logDatas[0]['stateEstimate.pwz']
                         ,]).T

    loadstatesDes = np.array([logDatas[0]['ctrltargetZ.x']/1000,
                              logDatas[0]['ctrltargetZ.y']/1000, 
                              logDatas[0]['ctrltargetZ.z']/1000,
                              logDatas[0]['ctrltargetZ.vx']/1000,
                              logDatas[0]['ctrltargetZ.vy']/1000, 
                              logDatas[0]['ctrltargetZ.vz']/1000,
                              logDatas[0]['ctrlLeeP.qp_desw'],
                              logDatas[0]['ctrlLeeP.qp_desx'],
                              logDatas[0]['ctrlLeeP.qp_desy'],
                              logDatas[0]['ctrlLeeP.qp_desz'],
                             ]).T

                            #   logDatas[0]['ctrlLeeP.omega_prx'],
                            #   logDatas[0]['ctrlLeeP.omega_pry'],
                            #   logDatas[0]['ctrlLeeP.omega_prz']

    if not np.isnan((loadstates[:,6:10])).any():
        fig1, fig2, fig3, fig4  = plotload(loadstates, loadstatesDes, time1)
    else: 
        fig1, fig2 = plotload(loadstates, loadstatesDes, time1)

    fig1.savefig(f, format='pdf', bbox_inches='tight')
    fig2.savefig(f, format='pdf', bbox_inches='tight')
    if not np.isnan((loadstates[:,6:10])).any():
        fig3.savefig(f, format='pdf', bbox_inches='tight')    
        fig4.savefig(f, format='pdf', bbox_inches='tight')  

    starttime = min([logDatas[k]['timestamp'][0] for k in range(len(files))])

    # payload position
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        p0 = np.array([logDatas[k]['stateEstimateZ.px']/1000,
                                logDatas[k]['stateEstimateZ.py']/1000, 
                                logDatas[k]['stateEstimateZ.pz']/1000,
                                ]).T

        p0d = np.array([logDatas[k]['ctrltargetZ.x']/1000,
                                logDatas[k]['ctrltargetZ.y']/1000, 
                                logDatas[k]['ctrltargetZ.z']/1000,
                                ]).T

        for i in range(0,3):
            ax[i].plot(time, p0[:,i], lw=0.75,label=filename)
            ax[i].plot(time, p0d[:,i], lw=0.75,label=filename + " desired")
    
    ax[0].set_ylabel('x [m]')
    ax[1].set_ylabel('y [m]')
    ax[2].set_ylabel('z [m]')
    ax[0].legend()
    fig.supxlabel("time [s]",fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(fig, grid[0, ::], 'Payload position')
    fig.savefig(f, format='pdf', bbox_inches='tight')

    # payload velocity
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        p0_dot = np.array([logDatas[k]['stateEstimateZ.pvx']/1000,
                                logDatas[k]['stateEstimateZ.pvy']/1000, 
                                logDatas[k]['stateEstimateZ.pvz']/1000,
                                ]).T

        p0 = np.array([logDatas[k]['stateEstimateZ.px']/1000,
                                logDatas[k]['stateEstimateZ.py']/1000, 
                                logDatas[k]['stateEstimateZ.pz']/1000,
                                ]).T

        p0_dot_est = np.diff(p0, axis=0) / np.tile(np.diff(time),(3,1)).T


        # numeric estimate


        for i in range(0,3):
            # ax[i].plot(time, p0_dot[:,i], lw=0.75,label=filename)
            ax[i].plot(time[1:], p0_dot_est[:,i], lw=0.75,label=filename + " finite diff")

    
    ax[0].set_ylabel('x [m/s]')
    ax[1].set_ylabel('y [m/s]')
    ax[2].set_ylabel('z [m/s]')
    ax[0].legend()
    fig.supxlabel("time [s]",fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(fig, grid[0, ::], 'Payload velocity')
    fig.savefig(f, format='pdf', bbox_inches='tight')


    # Fd
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    # compute common time horizon
    T = min([len(logDatas[k]['timestamp']) for k in range(len(files))])
    mu_sum = np.zeros((T, 3))
    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        Fd = np.array([logDatas[k]['ctrlLeeP.Fdx'],
                                logDatas[k]['ctrlLeeP.Fdy'], 
                                logDatas[k]['ctrlLeeP.Fdz'],
                                ]).T

        mu_sum += np.array([logDatas[k]['ctrlLeeP.desVirtInpx'],
                        logDatas[k]['ctrlLeeP.desVirtInpy'], 
                        logDatas[k]['ctrlLeeP.desVirtInpz'],
                        ]).T[0:T]

        for i in range(0,3):
            ax[i].plot(time, Fd[:,i], lw=0.75,label=filename)
    for i in range(0,3):
        ax[i].plot(time[0:T], mu_sum[:,i], lw=0.75,label="sum of mus")
    
    ax[0].set_ylabel('x [N]',)
    ax[1].set_ylabel('y [N]')
    ax[2].set_ylabel('z [N]')
    ax[0].legend()
    fig.supxlabel("time [s]",fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(fig, grid[0, ::], 'Fd')
    fig.savefig(f, format='pdf', bbox_inches='tight')

    # Md
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        Md = np.array([logDatas[k]['ctrlLeeP.Mdx'],
                                logDatas[k]['ctrlLeeP.Mdy'], 
                                logDatas[k]['ctrlLeeP.Mdz'],
                                ]).T
        for i in range(0,3):
            ax[i].plot(time, Md[:,i], lw=0.75,label=filename)
        ax[0].set_ylabel('x [Nm]',)
        ax[1].set_ylabel('y [Nm]')
        ax[2].set_ylabel('z [Nm]')
        ax[0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(3,1)
        create_subtitle(fig, grid[0, ::], 'Md')
    fig.savefig(f, format='pdf', bbox_inches='tight')

    # n1s's
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        n1 = np.array([logDatas[k]['ctrlLeeP.n1x'],
                                logDatas[k]['ctrlLeeP.n1y'], 
                                logDatas[k]['ctrlLeeP.n1z'],
                                ]).T

        n2 = np.array([logDatas[k]['ctrlLeeP.n2x'],
                                logDatas[k]['ctrlLeeP.n2y'], 
                                logDatas[k]['ctrlLeeP.n2z'],
                                ]).T

        for i in range(0,3):
            ax[i].plot(time, n1[:,i], lw=0.75,label=filename + " n1")
            ax[i].plot(time, n2[:,i], lw=0.75,label=filename + " n2")
        ax[0].set_ylabel('x [N]',)
        ax[1].set_ylabel('y [N]')
        ax[2].set_ylabel('z [N]')
        ax[0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(3,1)
        create_subtitle(fig, grid[0, ::], 'normals')
    fig.savefig(f, format='pdf', bbox_inches='tight')


    # mus
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        mu = np.array([logDatas[k]['ctrlLeeP.desVirtInpx'],
                                logDatas[k]['ctrlLeeP.desVirtInpy'], 
                                logDatas[k]['ctrlLeeP.desVirtInpz'],
                                ]).T
        for i in range(0,3):
            ax[i].plot(time, mu[:,i], lw=0.75,label=filename)
        ax[0].set_ylabel('x [N]',)
        ax[1].set_ylabel('y [N]')
        ax[2].set_ylabel('z [N]')
        ax[0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(3,1)
        create_subtitle(fig, grid[0, ::], 'mu')
    fig.savefig(f, format='pdf', bbox_inches='tight')

    # u's
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        mu = np.array([logDatas[k]['ctrlLeeP.ux'],
                                logDatas[k]['ctrlLeeP.uy'], 
                                logDatas[k]['ctrlLeeP.uz'],
                                ]).T
        for i in range(0,3):
            ax[i].plot(time, mu[:,i], lw=0.75,label=filename)
        ax[0].set_ylabel('x [N]',)
        ax[1].set_ylabel('y [N]')
        ax[2].set_ylabel('z [N]')
        ax[0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(3,1)
        create_subtitle(fig, grid[0, ::], 'u')
    fig.savefig(f, format='pdf', bbox_inches='tight')


    # cable length
    fig, ax = plt.subplots(1, 1, sharex=True,squeeze=False)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        pi = np.array([ logDatas[k]['stateEstimateZ.x']/1000,
                            logDatas[k]['stateEstimateZ.y']/1000, 
                            logDatas[k]['stateEstimateZ.z']/1000]).T

        p0 = np.array([ logDatas[k]['stateEstimateZ.px']/1000,
                            logDatas[k]['stateEstimateZ.py']/1000, 
                            logDatas[k]['stateEstimateZ.pz']/1000]).T

        q = np.array([logDatas[k]['stateEstimate.pqw'],
                            logDatas[k]['stateEstimate.pqx'],
                            logDatas[k]['stateEstimate.pqy'],
                            logDatas[k]['stateEstimate.pqz']]).T

        plStPos = p0 + rn.rotate(q, att_points[k])

        li = np.linalg.norm(pi - plStPos, axis=1)
        ax[0,0].plot(time, li, lw=0.75,label=filename)
        ax[0,0].plot(time, li, lw=0.75,label=filename)
        # ax[0].set_ylabel('x [N]',)
        # ax[1].set_ylabel('y [N]')
        # ax[2].set_ylabel('z [N]')
        ax[0,0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(1,1)
        create_subtitle(fig, grid[0, ::], 'cable length')
    fig.savefig(f, format='pdf', bbox_inches='tight')

    # # qidot
    # fig, ax = plt.subplots(3, 1, sharex=True)
    # fig.tight_layout()

    # for k, filename in enumerate(files):
    #     time = (logDatas[k]['timestamp']-logDatas[k]['timestamp'][0])/1000

    #     pi_dot = np.array([ logDatas[k]['stateEstimateZ.vx']/1000,
    #                         logDatas[k]['stateEstimateZ.vy']/1000, 
    #                         logDatas[k]['stateEstimateZ.vz']/1000]).T

    #     p0_dot = np.array([ logDatas[k]['stateEstimateZ.pvx']/1000,
    #                         logDatas[k]['stateEstimateZ.pvy']/1000, 
    #                         logDatas[k]['stateEstimateZ.pvz']/1000]).T

    #     pi = np.array([ logDatas[k]['stateEstimateZ.x']/1000,
    #                         logDatas[k]['stateEstimateZ.y']/1000, 
    #                         logDatas[k]['stateEstimateZ.z']/1000]).T

    #     p0 = np.array([ logDatas[k]['stateEstimateZ.px']/1000,
    #                         logDatas[k]['stateEstimateZ.py']/1000, 
    #                         logDatas[k]['stateEstimateZ.pz']/1000]).T

    #     q = np.array([logDatas[k]['stateEstimate.pqw'],
    #                         logDatas[k]['stateEstimate.pqx'],
    #                         logDatas[k]['stateEstimate.pqy'],
    #                         logDatas[k]['stateEstimate.pqz']]).T

    #     plStPos = p0 + rn.rotate(q, att_points[k])

    #     li = np.linalg.norm(pi - plStPos, axis=1)
    #     qi_dot = p0_dot - pi_dot
    #     for t in range(li.shape[0]):
    #         qi_dot[t] /= li[t]

    #     qi_dot_fw = np.array([logDatas[k]['ctrlLeeP.qidotx'],
    #                         logDatas[k]['ctrlLeeP.qidoty'],
    #                         logDatas[k]['ctrlLeeP.qidotz']]).T

    #     # qi_dot = np.divide(p0_dot - pi_dot, li)

    #     for i in range(0,3):
    #         ax[i].plot(time, qi_dot[:,i], lw=0.75,label=filename)
    #         ax[i].plot(time, qi_dot_fw[:,i], lw=0.75,label=filename + " FW")
    #     ax[0].set_ylabel('x [N]',)
    #     ax[1].set_ylabel('y [N]')
    #     ax[2].set_ylabel('z [N]')
    #     ax[0].legend()
    #     fig.supxlabel("time [s]",fontsize='small')
    #     grid = plt.GridSpec(3,1)
    #     create_subtitle(fig, grid[0, ::], 'qidot')
    # fig.savefig(f, format='pdf', bbox_inches='tight')

    # cable states
    mucf5 = np.array([
      logDatas[0]['ctrlLeeP.desVirtInpx'], logDatas[0]['ctrlLeeP.desVirtInpy'], logDatas[0]['ctrlLeeP.desVirtInpz'] ]).T
    qd1 = []
    for i in range(mucf5.shape[0]):
        n_mucf5 = np.linalg.norm(mucf5[i,:])
        qd1.append(-mucf5[i,:]/n_mucf5)
    qd1 = np.array(qd1).reshape(mucf5.shape[0],3)
  
    cablestates1 = np.array([ logDatas[0]['ctrlLeeP.qix'],
                              logDatas[0]['ctrlLeeP.qiy'], 
                              logDatas[0]['ctrlLeeP.qiz'],
                              logDatas[0]['ctrlLeeP.qidotx'],
                              logDatas[0]['ctrlLeeP.qidoty'], 
                              logDatas[0]['ctrlLeeP.qidotz'],
                              logDatas[0]['ctrlLeeP.qdidotx'],
                              logDatas[0]['ctrlLeeP.qdidoty'],
                              logDatas[0]['ctrlLeeP.qdidotz'],
                            ]).T
    mucf6 = np.array([
      logDatas[1]['ctrlLeeP.desVirtInpx'], logDatas[1]['ctrlLeeP.desVirtInpy'], logDatas[1]['ctrlLeeP.desVirtInpz'] ]).T
    
    n_mucf6 = np.linalg.norm(mucf6, axis=1)
    qd2 = []
    for i in range(mucf6.shape[0]):
        n_mucf6 = np.linalg.norm(mucf6[i,:])
        qd2.append(-mucf6[i,:]/n_mucf6)
    qd2 = np.array(qd2).reshape(mucf6.shape[0],3)
  
    cablestates2 = np.array([ logDatas[1]['ctrlLeeP.qix'],
                              logDatas[1]['ctrlLeeP.qiy'], 
                              logDatas[1]['ctrlLeeP.qiz'],
                              logDatas[1]['ctrlLeeP.qidotx'],
                              logDatas[1]['ctrlLeeP.qidoty'],
                              logDatas[1]['ctrlLeeP.qidotz'],
                              logDatas[1]['ctrlLeeP.qdidotx'],
                              logDatas[1]['ctrlLeeP.qdidoty'],
                              logDatas[1]['ctrlLeeP.qdidotz'],
                            ]).T

    time2 = (logDatas[1]['timestamp'][-1]-logDatas[1]['timestamp'][0])/1000
    time2 = np.linspace(0, time2, num=len(logDatas[1]['stateEstimateZ.px']))

   

    cf5 = np.array([ logDatas[0]['stateEstimateZ.x']/1000,
                     logDatas[0]['stateEstimateZ.y']/1000, 
                     logDatas[0]['stateEstimateZ.z']/1000,
                     logDatas[0]['stateEstimateZ.vx']/1000,
                     logDatas[0]['stateEstimateZ.vy']/1000, 
                     logDatas[0]['stateEstimateZ.vz']/1000,
                     logDatas[0]['stateEstimate.qw'],
                     logDatas[0]['stateEstimate.qx'],
                     logDatas[0]['stateEstimate.qy'],
                     logDatas[0]['stateEstimate.qz'],
                     np.radians(logDatas[0]['gyro.x']),
                     np.radians(logDatas[0]['gyro.y']),
                     np.radians(logDatas[0]['gyro.z']),
                         ]).T
    quatdcf5 = rn.from_euler(logDatas[0]['ctrlLeeP.rpydx'], 
                             logDatas[0]['ctrlLeeP.rpydy'], 
                             logDatas[0]['ctrlLeeP.rpydz'], convention='xyz')
    
    cf5des = np.array([quatdcf5[:,0],quatdcf5[:,1], quatdcf5[:,2], quatdcf5[:,3],
                        logDatas[0]['ctrlLeeP.omegarx'],
                        logDatas[0]['ctrlLeeP.omegary'], 
                        logDatas[0]['ctrlLeeP.omegarz'],
                      ]).T

    cf6 = np.array([logDatas[1]['stateEstimateZ.x']/1000,
                    logDatas[1]['stateEstimateZ.y']/1000, 
                    logDatas[1]['stateEstimateZ.z']/1000,
                    logDatas[1]['stateEstimateZ.vx']/1000,
                    logDatas[1]['stateEstimateZ.vy']/1000, 
                    logDatas[1]['stateEstimateZ.vz']/1000,
                    logDatas[1]['stateEstimate.qw'],
                    logDatas[1]['stateEstimate.qx'],
                    logDatas[1]['stateEstimate.qy'],
                    logDatas[1]['stateEstimate.qz'],
                    np.radians(logDatas[1]['gyro.x']),
                    np.radians(logDatas[1]['gyro.y']),
                    np.radians(logDatas[1]['gyro.z'])]).T

    quatdcf6 = rn.from_euler(logDatas[1]['ctrlLeeP.rpydx'], 
                             logDatas[1]['ctrlLeeP.rpydy'], 
                             logDatas[1]['ctrlLeeP.rpydz'], convention='xyz')
    
    cf6des = np.array([quatdcf6[:,0],quatdcf6[:,1], quatdcf6[:,2], quatdcf6[:,3],
                        logDatas[1]['ctrlLeeP.omegarx'],
                        logDatas[1]['ctrlLeeP.omegary'], 
                        logDatas[1]['ctrlLeeP.omegarz'],
                      ]).T
   
    fig5, fig6 = plotcable(cablestates1, qd1, time1, 'cf5')
    fig7, fig8 = plotcable(cablestates2, qd2, time2, 'cf6')
    fig9, fig10, fig11, figv1 = plotuav(cf5, cf5des,time1, 'cf5')
    fig12, fig13, fig14, figv2 = plotuav(cf6, cf6des, time2, 'cf6')
   
  
    fig5.savefig(f, format='pdf', bbox_inches='tight')        
    fig6.savefig(f, format='pdf', bbox_inches='tight')        
    fig7.savefig(f, format='pdf', bbox_inches='tight')        
    fig8.savefig(f, format='pdf', bbox_inches='tight')        
    fig9.savefig(f, format='pdf', bbox_inches='tight')        
    fig10.savefig(f, format='pdf', bbox_inches='tight')        
    fig11.savefig(f, format='pdf', bbox_inches='tight')        
    figv1.savefig(f, format='pdf', bbox_inches='tight')        
    fig12.savefig(f, format='pdf', bbox_inches='tight')        
    fig13.savefig(f, format='pdf', bbox_inches='tight')        
    fig14.savefig(f, format='pdf', bbox_inches='tight')        
    figv2.savefig(f, format='pdf', bbox_inches='tight')        
    f.close()


    Fd5 = np.array([
      logDatas[0]['ctrlLeeP.Fdx']  , logDatas[0]['ctrlLeeP.Fdy'], logDatas[0]['ctrlLeeP.Fdz']
    ]).T


    size1 = len(logDatas[0]['ctrlLeeP.n1x'])
    cf5hp = np.array([
        logDatas[0]['ctrlLeeP.n1x'], logDatas[0]['ctrlLeeP.n1y'], logDatas[0]['ctrlLeeP.n1z'], np.zeros(size1,) 
    ]).T
 
    
    Fd6  = np.array([
      logDatas[1]['ctrlLeeP.Fdx']  , logDatas[1]['ctrlLeeP.Fdy'], logDatas[1]['ctrlLeeP.Fdz']
    ]).T
    
    size2 = len(logDatas[1]['ctrlLeeP.n1x'])
    cf6hp = np.array([
        logDatas[1]['ctrlLeeP.n1x'], logDatas[1]['ctrlLeeP.n1y'], logDatas[1]['ctrlLeeP.n1z'], np.zeros(size2,) 
    ]).T

    # check that all arrays have the same size
    while np.size(mucf5) != np.size(mucf6):
        if np.size(mucf5) < np.size(mucf6):
            mucf5 =  np.append(mucf5, [mucf5[-1,:]], axis=0)
        elif np.size(mucf5) > np.size(mucf6):
            mucf6 = np.append(mucf6, [mucf6[-1,:]], axis=0)


    while np.size(cf5hp) != np.size(cf6hp):
        if np.size(cf5hp) < np.size(cf6hp):
            cf5hp =  np.append(cf5hp, [cf5hp[-1,:]], axis=0)
        elif np.size(cf5hp) > np.size(cf6hp):
            cf6hp = np.append(cf6hp, [cf6hp[-1,:]], axis=0)
   
    while np.size(Fd5) != np.size(Fd6):
        if np.size(Fd5) < np.size(Fd6):
            Fd5 =  np.append(Fd5, [Fd5[-1,:]], axis=0)
        elif np.size(Fd5) > np.size(Fd6):
            Fd6 = np.append(Fd6, [Fd6[-1,:]], axis=0)
    

    while np.size(cf5) != np.size(cf6):
        if np.size(cf5) < np.size(cf6):
            cf5 =  np.append(cf5, [cf5[-1,:]], axis=0)
        elif np.size(cf5) > np.size(cf6):
            cf6 = np.append(cf6, [cf6[-1,:]], axis=0)


    with open("../../sim/output/payload.csv", "w") as f:
            np.savetxt(f,loadstates, delimiter=",")
    
    with open("../../sim/output/cf5.csv", "w") as f:
            np.savetxt(f,cf5, delimiter=",")    

    with open("../../sim/output/Fd5.csv", "w") as f:
            np.savetxt(f,Fd5, delimiter=",")

    with open("../../sim/output/Fd6.csv", "w") as f:
        np.savetxt(f,Fd6, delimiter=",")
        
    with open("../../sim/output/cf6.csv", "w") as f:
            np.savetxt(f,cf6, delimiter=",")    

        
    with open('../../sim/output/mu_cf5.csv', "w") as f:
        np.savetxt(f, mucf5, delimiter=",")


    with open('../../sim/output/mu_cf6.csv', "w") as f:
        np.savetxt(f, mucf6, delimiter=",")

    with open('../../sim/output/hp1_cf5.csv', "w") as f:
        np.savetxt(f, cf5hp, delimiter=",")

    with open('../../sim/output/hp1_cf6.csv', "w") as f:
        np.savetxt(f, cf6hp, delimiter=",")
    
    robots = {
        'cf5': {
        'att': att_points[0],
            'hps': ['hp1_cf5.csv'],
            'mu' : 'mu_cf5.csv',
            'state': 'cf5.csv',
            'Fd': 'Fd5.csv'
        },
        'cf6' : {
        'att': att_points[1],
        'hps': ['hp1_cf6.csv'],
        'mu' : 'mu_cf6.csv',
        'state': 'cf6.csv',
        'Fd': 'Fd6.csv'
        }
    }
    configData['robots'] = robots

    with open("../../sim/output/configData.yaml", 'w') as f:
            yaml.dump(configData, f)
if __name__ == '__main__':
    main()