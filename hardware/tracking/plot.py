import cfusdlog
import matplotlib.pyplot as plt
import numpy as np
import rowan
# from matplotlib.backends.backend_pdf import PdfPages


def main(args=None):

    # files = ["cf4_77", "cf6_80", "cf7_32"]
    files = ["cf4_86", "cf6_86"]

    logDatas = [cfusdlog.decode(f)['fixedFrequency'] for f in files]
    
    starttime = min([logDatas[k]['timestamp'][0] for k in range(len(files))])

    # filter
    for k in range(len(files)):
        t = (logDatas[k]['timestamp'] - starttime)/1000
        idx = np.where(np.logical_and(t > 21.0, t < 37.0))
        for key, value in logDatas[k].items():
            logDatas[k][key] = value[idx]

    # re-compute start time
#     starttime = min([logDatas[k]['timestamp'][0] for k in range(len(files))])
    

    # payload position error
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

        error = p0 - p0d
        eucl_error = np.linalg.norm(error, axis=1)*100 # in cm
        print(eucl_error)
        print("Position error stats: {} cm ({})".format(np.mean(eucl_error), np.std(eucl_error)) )

        for i in range(0,3):
            ax[i].plot(time, error[:,i], lw=0.75,label=filename)
    
    ax[0].set_ylabel('x [m]')
    ax[1].set_ylabel('y [m]')
    ax[2].set_ylabel('z [m]')
    ax[0].legend()
    fig.supxlabel("time [s]",fontsize='small')
    plt.show()

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
    plt.show()

    # payload velocity
    fig, ax = plt.subplots(3, 1, sharex=True)
    fig.tight_layout()

    for k, filename in enumerate(files):
        time = (logDatas[k]['timestamp']-starttime)/1000

        v0 = np.array([logDatas[k]['stateEstimateZ.pvx']/1000,
                                logDatas[k]['stateEstimateZ.pvy']/1000, 
                                logDatas[k]['stateEstimateZ.pvz']/1000,
                                ]).T

        v0d = np.array([logDatas[k]['ctrltargetZ.vx']/1000,
                                logDatas[k]['ctrltargetZ.vy']/1000, 
                                logDatas[k]['ctrltargetZ.vz']/1000,
                                ]).T


        for i in range(0,3):
            ax[i].plot(time, v0[:,i], lw=0.75,label=filename)
            ax[i].plot(time, v0d[:,i], lw=0.75,label=filename + " desired")


    
    ax[0].set_ylabel('x [m/s]')
    ax[1].set_ylabel('y [m/s]')
    ax[2].set_ylabel('z [m/s]')
    ax[0].legend()
    fig.supxlabel("time [s]",fontsize='small')

    plt.show()

    # Payload Orientation
    fig, ax = plt.subplots(3, 1, sharex=True)
    
    for file, logData in zip(files, logDatas):
        time = (logData['timestamp']-starttime)/1000

        q = np.array([ 
            logData['stateEstimate.pqw'],
            logData['stateEstimate.pqx'],
            logData['stateEstimate.pqy'],
            logData['stateEstimate.pqz']]).T
        rpy = np.degrees(rowan.to_euler(rowan.normalize(q), convention='xyz'))

        qp_des = np.array([ 
            logData['ctrlLeeP.qp_desw'],
            logData['ctrlLeeP.qp_desx'],
            logData['ctrlLeeP.qp_desy'],
            logData['ctrlLeeP.qp_desz']]).T
        
        rpydes = np.degrees(rowan.to_euler(rowan.normalize(qp_des), convention='xyz'))
        
        for i in range(0,3):
            ax[i].plot(time, rpy[:,i], lw=0.75,label=file)
            ax[i].plot(time, rpydes[:,i], lw=0.75,label=file + " desired")

        # error_quat = rowan.geometry.intrinsic_distance(q, qp_des)

    ax[0].set_ylabel('x [deg]')
    ax[1].set_ylabel('y [deg]')
    ax[2].set_ylabel('z [deg]')
    ax[0].legend()
    fig.supxlabel("time [s]",fontsize='small')

    plt.show()        
    exit()

    # Payload Orientation
    fig, axs = plt.subplots(3, 1, sharex=True)
    axs[0].set_ylabel("x [deg]")
    axs[1].set_ylabel("y [deg]")
    axs[2].set_ylabel("z [deg]")
    axs[-1].set_xlabel("Time [ms]")
    
    for file, logData in zip(files, logDatas):
        time = (logData['timestamp']-starttime)/1000

        q = np.array([ 
                logData['stateEstimate.qw'],
                logData['stateEstimate.qx'],
                logData['stateEstimate.qy'],
                logData['stateEstimate.qz']]).T

        
        q = np.array([ 
        logData['stateEstimate.pqw'],
        logData['stateEstimate.pqx'],
        logData['stateEstimate.pqy'],
        logData['stateEstimate.pqz']]).T

        qp_des = np.array([ 
        logData['ctrlLeeP.qp_desw'],
        logData['ctrlLeeP.qp_desx'],
        logData['ctrlLeeP.qp_desy'],
        logData['ctrlLeeP.qp_desz']]).T
        
        
        for i in range(0,3):
            ax[i].plot(time, rpy[:,i], lw=0.75,label=filename)
            ax[i].plot(time, rpydes[:,i], lw=0.75,label=filename + " desired")

        error_quat = rowan.geometry.intrinsic_distance(q, qp_des)


    exit()

    # Payload orientation
    fig, axs = plt.subplots(3, 1, sharex=True)
    axs[0].set_ylabel("x [deg]")
    axs[1].set_ylabel("y [deg]")
    axs[2].set_ylabel("z [deg]")
    axs[-1].set_xlabel("Time [ms]")
    for file, logData in zip(files, logDatas):

        q = np.array([ 
                logData['stateEstimate.qw'],
                logData['stateEstimate.qx'],
                logData['stateEstimate.qy'],
                logData['stateEstimate.qz']]).T

        rpy = np.degrees(rowan.to_euler(rowan.normalize(q), convention='xyz'))
        
        axs[0].plot(logData['timestamp'], rpy[:,0], label=file)
        axs[1].plot(logData['timestamp'], rpy[:,1], label=file)
        axs[2].plot(logData['timestamp'], rpy[:,2], label=file)

        q = np.array([ 
        logData['stateEstimate.pqw'],
        logData['stateEstimate.pqx'],
        logData['stateEstimate.pqy'],
        logData['stateEstimate.pqz']]).T

        qp_des = np.array([ 
        logData['ctrlLeeP.qp_desw'],
        logData['ctrlLeeP.qp_desx'],
        logData['ctrlLeeP.qp_desy'],
        logData['ctrlLeeP.qp_desz']]).T
        rpydes = np.degrees(rowan.to_euler(rowan.normalize(q), convention='xyz'))
        

        axs[0].plot(logData['timestamp']-latency, rpy[:,0], label=file + " payload")
        axs[1].plot(logData['timestamp']-latency, rpy[:,1], label=file + " payload")
        axs[2].plot(logData['timestamp']-latency, rpy[:,2], label=file + " payload")

    axs[-1].legend()

    plt.show()
    # exit()

    # Payload omega
    fig, axs = plt.subplots(3, 1, sharex=True)
    axs[0].set_ylabel("x [deg/s]")
    axs[1].set_ylabel("y [deg/s]")
    axs[2].set_ylabel("z [deg/s]")
    axs[-1].set_xlabel("Time [ms]")
    for file, logData in zip(files, logDatas):

        w = np.array([ 
                logData['gyro.x'],
                logData['gyro.y'],
                logData['gyro.z']]).T
        
        axs[0].plot(logData['timestamp'], w[:,0], label=file)
        axs[1].plot(logData['timestamp'], w[:,1], label=file)
        axs[2].plot(logData['timestamp'], w[:,2], label=file)

        w = np.degrees(np.array([ 
                logData['stateEstimate.pwx'],
                logData['stateEstimate.pwy'],
                logData['stateEstimate.pwz']]).T)

        axs[0].plot(logData['timestamp']-latency, w[:,0], label=file + " payload")
        axs[1].plot(logData['timestamp']-latency, w[:,1], label=file + " payload")
        axs[2].plot(logData['timestamp']-latency, w[:,2], label=file + " payload")

    axs[-1].legend()

    plt.show()


if __name__ == '__main__':
    main()