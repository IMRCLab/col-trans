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


def skew(w):
    w = np.asarray(w).reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))

def create_subtitle(fig: plt.Figure, grid: SubplotSpec, title: str):
    row = fig.add_subplot(grid)
    row.set_title('\n\n\n'+title, fontweight='medium',fontsize='medium')
    row.set_frame_on(False)
    row.axis('off')

def main(args=None):
    
    files = ["cf4_86"]
    att_points = [[0,0.07,0], [0,-0.07,0]]
    shape = 'cuboid'

    # files = ["../tracking/cf6_77", "../tracking/cf7_29"]
    # att_points = [[0,0.0,0], [0,0.0,0], [0,0.0,0]]
    # shape = 'point'

    # files = ["cf4_27", "cf5_27", "cf6_27"]
    # att_points = [[-0.02, 0.035, 0.0], [0.04, 0.0, 0.0], [-0.02, -0.035, 0.0]]
    # shape = 'triangle'

    # files = ["../tracking/cf4_74", "../tracking/cf4_77", "../tracking/cf7_29"]
    # att_points = [[0,0.0,0], [0,0.0,0]]
    # shape = 'point'


    logDatas = [cfusdlog.decode(f)['fixedFrequency'] for f in files]

    starttime = min([logDatas[k]['timestamp'][0] for k in range(len(files))])
    # starttime = 0
    # print(logDatas[1]['timestamp'])
    # exit()

    for k in range(len(files)):
        idx = np.where((logDatas[k]['timestamp'] - starttime)/1000 < 36)
        for key, value in logDatas[k].items():
            logDatas[k][key] = value[idx]

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

        # p0_dot_est = np.diff(p0, axis=0) / np.tile(np.diff(time),(3,1)).T

        for i in range(0,3):
            ax[i].plot(time, p0_dot[:,i], lw=0.75,label=filename)
            # ax[i].plot(time[1:], p0_dot_est[:,i], lw=0.75,label=filename + " finite diff")

    
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

    Md_sum = np.zeros((T, 3))
    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        Md = np.array([logDatas[k]['ctrlLeeP.Mdx'],
                                logDatas[k]['ctrlLeeP.Mdy'], 
                                logDatas[k]['ctrlLeeP.Mdz'],
                                ]).T

        mu = np.array([logDatas[k]['ctrlLeeP.desVirtInpx'],
                        logDatas[k]['ctrlLeeP.desVirtInpy'], 
                        logDatas[k]['ctrlLeeP.desVirtInpz'],
                        ]).T[0:T]

        q = np.array([logDatas[k]['stateEstimate.pqw'],
                    logDatas[k]['stateEstimate.pqx'],
                    logDatas[k]['stateEstimate.pqy'],
                    logDatas[k]['stateEstimate.pqz']]).T[0:T]
        Rp = rn.to_matrix(q, False)
        for t in range(T):
            Md_sum[t] += skew(att_points[k]) @ Rp[t].T @ mu[t]

        for i in range(0,3):
            ax[i].plot(time, Md[:,i], lw=0.75,label=filename)

    for i in range(0,3):
        ax[i].plot(time[0:T], Md_sum[:,i], lw=0.75,label="sum of mus")
    
    ax[0].set_ylabel('x [Nm]',)
    ax[1].set_ylabel('y [Nm]')
    ax[2].set_ylabel('z [Nm]')
    ax[0].legend()
    fig.supxlabel("time [s]",fontsize='small')
    grid = plt.GridSpec(3,1)
    create_subtitle(fig, grid[0, ::], 'Md')
    fig.savefig(f, format='pdf', bbox_inches='tight')

    # n1s's
    # fig, ax = plt.subplots(3, 1, sharex=True)
    # fig.tight_layout()

    # for k, filename in enumerate(files):
    #     time = (logDatas[k]['timestamp']-starttime)/1000

    #     n1 = np.array([logDatas[k]['ctrlLeeP.n1x'],
    #                             logDatas[k]['ctrlLeeP.n1y'], 
    #                             logDatas[k]['ctrlLeeP.n1z'],
    #                             ]).T

    #     n2 = np.array([logDatas[k]['ctrlLeeP.n2x'],
    #                             logDatas[k]['ctrlLeeP.n2y'], 
    #                             logDatas[k]['ctrlLeeP.n2z'],
    #                             ]).T

    #     for i in range(0,3):
    #         ax[i].plot(time, n1[:,i], lw=0.75,label=filename + " n1")
    #         ax[i].plot(time, n2[:,i], lw=0.75,label=filename + " n2")
    #     ax[0].set_ylabel('x [N]',)
    #     ax[1].set_ylabel('y [N]')
    #     ax[2].set_ylabel('z [N]')
    #     ax[0].legend()
    #     fig.supxlabel("time [s]",fontsize='small')
    #     grid = plt.GridSpec(3,1)
    #     create_subtitle(fig, grid[0, ::], 'normals')
    # fig.savefig(f, format='pdf', bbox_inches='tight')


    # mus
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        mu1 = np.array([logDatas[k]['ctrlLeeP.desVirtInpx'],
                                logDatas[k]['ctrlLeeP.desVirtInpy'], 
                                logDatas[k]['ctrlLeeP.desVirtInpz'],
                                ]).T

        # mu2 = np.array([logDatas[k]['ctrlLeeP.desVirtInp2x'],
        #                         logDatas[k]['ctrlLeeP.desVirtInp2y'], 
        #                         logDatas[k]['ctrlLeeP.desVirtInp2z'],
        #                         ]).T

        for i in range(0,3):
            ax[i].plot(time, mu1[:,i], lw=0.75,label=filename +" mu1")
            # ax[i].plot(time, mu2[:,i], lw=0.75,label=filename +" mu2")
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
        # ax[0,0].plot(time, li, lw=0.75,label=filename)
        # ax[0].set_ylabel('x [N]',)
        # ax[1].set_ylabel('y [N]')
        # ax[2].set_ylabel('z [N]')
        ax[0,0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(1,1)
        create_subtitle(fig, grid[0, ::], 'cable length')
    fig.savefig(f, format='pdf', bbox_inches='tight')



    fig, ax = plt.subplots(3,1, sharex=True)
    fig.tight_layout()
    # uav rpy vs rpydes 
    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        q = np.array([logDatas[k]['stateEstimate.qw'],
                     logDatas[k]['stateEstimate.qx'],
                     logDatas[k]['stateEstimate.qy'],
                     logDatas[k]['stateEstimate.qz']]).T
        rpy = np.degrees(rn.to_euler(rn.normalize(q), convention='xyz'))

        rpydes = np.degrees(np.array([logDatas[k]['ctrlLeeP.rpydx'],
                           logDatas[k]['ctrlLeeP.rpydy'],
                           logDatas[k]['ctrlLeeP.rpydz']])).T

        for i in range(0,3):
            ax[i].plot(time, rpy[:,i], lw=0.75,label=filename +" rpy")
            ax[i].plot(time, rpydes[:,i], lw=0.75,label=filename +" rpydes")
        ax[0].set_ylabel('x [rad]',)
        ax[1].set_ylabel('y [rad]')
        ax[2].set_ylabel('z [rad]')
        ax[0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(3,1)
        create_subtitle(fig, grid[0, ::], 'rpy')
    fig.savefig(f, format='pdf', bbox_inches='tight')


    fig, ax = plt.subplots(3,1, sharex=True)
    fig.tight_layout()
    # uav gyro vs omegar 
    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        omega = np.array([np.radians(logDatas[k]['gyro.x']),
                          np.radians(logDatas[k]['gyro.y']),
                          np.radians(logDatas[k]['gyro.z'])]).T

        omegar = np.array([logDatas[k]['ctrlLeeP.omegarx'],
                          logDatas[k]['ctrlLeeP.omegary'],
                          logDatas[k]['ctrlLeeP.omegarz']]).T

        for i in range(0,3):
            ax[i].plot(time, omega[:,i], lw=0.75,label=filename +" rpy")
            ax[i].plot(time, omegar[:,i], lw=0.75,label=filename +" rpydes")
        ax[0].set_ylabel('x [rad/s]',)
        ax[1].set_ylabel('y [rad/s]')
        ax[2].set_ylabel('z [rad/s]')
        ax[0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(3,1)
        create_subtitle(fig, grid[0, ::], 'rpy')
    fig.savefig(f, format='pdf', bbox_inches='tight')


    fig, ax = plt.subplots(3,1, sharex=True)
    fig.tight_layout()
    # uav thrust vs maxthrust [g]
    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        thrustpart = np.array([logDatas[k]['powerDist.thrustPart']/(100)])
        rollpart   = np.array([logDatas[k]['powerDist.rollPart']/(100)])
        pitchpart  = np.array([logDatas[k]['powerDist.pitchPart']/(100)])
        yawpart    = np.array([logDatas[k]['powerDist.yawPart']/(100)])
        maxThrust  = np.array([logDatas[k]['powerDist.maxThrust']/(100)])

        motorForces = np.row_stack(( 
        thrust - roll - pitch + yaw,
        thrust - roll + pitch - yaw,
        thrust + roll + pitch + yaw,
        thrust + roll - pitch - yaw
        )).T 

        for i in range(0,4):
            ax[i].plot(time, motorForces[:,i], lw=0.75,label=filename +" motorForces")
            ax[i].plot(time, maxThrust[i], lw=0.75,label=filename +" maxThrust")
        ax[0].set_ylabel('m1 [g]',)
        ax[1].set_ylabel('m2 [g]')
        ax[2].set_ylabel('m3 [g]')
        ax[3].set_ylabel('m4 [g]')
        ax[0].legend()
        fig.supxlabel("time [s]",fontsize='small')
        grid = plt.GridSpec(3,1)
        create_subtitle(fig, grid[0, ::], 'motor forces')
    fig.savefig(f, format='pdf', bbox_inches='tight')
            
    f.close()
if __name__ == '__main__':
    main()