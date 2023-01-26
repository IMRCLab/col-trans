import cfusdlog
import matplotlib.pyplot as plt
import numpy as np
import rowan
# from matplotlib.backends.backend_pdf import PdfPages


def main(args=None):

    files = ["cf4_40"]
    latency = 15 # guess in ms

    logDatas = [cfusdlog.decode(f)['fixedFrequency'] for f in files]

    # Payload omega
    fig, axs = plt.subplots(3, 1, sharex=True)
    axs[0].set_ylabel("x [m/s]")
    axs[1].set_ylabel("y [m/s]")
    axs[2].set_ylabel("z [m/s]")
    axs[-1].set_xlabel("Time [ms]")
    for file, logData in zip(files, logDatas):

        v = np.array([ 
                logData['stateEstimateZ.x'] / 1000.0,
                logData['stateEstimateZ.y'] / 1000.0,
                logData['stateEstimateZ.z'] / 1000.0]).T
        
        axs[0].plot(logData['timestamp'], v[:,0], label=file)
        axs[1].plot(logData['timestamp'], v[:,1], label=file)
        axs[2].plot(logData['timestamp'], v[:,2], label=file)

        w = np.degrees(np.array([ 
                logData['stateEstimateZ.pvx'] / 1000.0,
                logData['stateEstimateZ.pvy'] / 1000.0,
                logData['stateEstimateZ.pvz'] / 1000.0]).T)

        axs[0].plot(logData['timestamp']-latency, v[:,0], label=file + " payload")
        axs[1].plot(logData['timestamp']-latency, v[:,1], label=file + " payload")
        axs[2].plot(logData['timestamp']-latency, v[:,2], label=file + " payload")

    axs[-1].legend()

    plt.show()
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

        rpy = np.degrees(rowan.to_euler(rowan.normalize(q), convention='xyz'))
        
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