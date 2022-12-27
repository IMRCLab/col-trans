import cfusdlog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def main(args=None):

    files = ["cf231_00", "cf5_00", "cf6_00"]

    logDatas = [cfusdlog.decode(f)['fixedFrequency'] for f in files]


    with PdfPages('result.pdf') as pdf:

        # Payload position
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("px [m]")
        axs[1].set_ylabel("py [m]")
        axs[2].set_ylabel("pz [m]")
        axs[-1].set_xlabel("Time [s]")
        for file, logData in zip(files, logDatas):
            axs[0].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.px'], label=file)
            axs[1].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.py'], label=file)
            axs[2].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.pz'], label=file)
        pdf.savefig(fig)
        plt.close()

        # Payload velocity
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("pvx [m/s]")
        axs[1].set_ylabel("pvy [m/s]")
        axs[2].set_ylabel("pvz [m/s]")
        axs[-1].set_xlabel("Time [s]")
        for file, logData in zip(files, logDatas):
            axs[0].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.pvx'], label=file)
            axs[1].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.pvy'], label=file)
            axs[2].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.pvz'], label=file)
        pdf.savefig(fig)
        plt.close()

        # Fd
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("Fdx")
        axs[1].set_ylabel("Fdy")
        axs[2].set_ylabel("Fdz")
        axs[-1].set_xlabel("Time [s]")
        for file, logData in zip(files, logDatas):
            axs[0].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.Fdx'], label=file)
            axs[1].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.Fdy'], label=file)
            axs[2].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.Fdz'], label=file)
        pdf.savefig(fig)
        plt.close()

        # Fd
        fig, axs = plt.subplots(3, 1, sharex=True)
        axs[0].set_ylabel("desVirtInpx")
        axs[1].set_ylabel("desVirtInpy")
        axs[2].set_ylabel("desVirtInpz")
        axs[-1].set_xlabel("Time [s]")
        for file, logData in zip(files, logDatas):
            axs[0].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.desVirtInpx'], label=file)
            axs[1].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.desVirtInpy'], label=file)
            axs[2].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.desVirtInpz'], label=file)
        pdf.savefig(fig)
        plt.close()

        # axs[3].set_ylabel("pvx [m/s]")
        # axs[4].set_ylabel("pvy [m/s]")
        # axs[5].set_ylabel("pvz [m/s]")
        # axs[-1].set_xlabel("Time [s]")

        # for file in files:
        #     # decode binary log data
        #     logData = cfusdlog.decode(file)

        #     #only focus on regular logging
        #     logData = logData['fixedFrequency']

        #     axs[0].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.Fdx'], label=file)
        #     axs[1].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.Fdy'], label=file)
        #     axs[2].plot(logData['timestamp'] / 1000.0, logData['ctrlLeeP.Fdz'], label=file)
        #     axs[3].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.pvx'], label=file)
        #     axs[4].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.pvy'], label=file)
        #     axs[5].plot(logData['timestamp'] / 1000.0, logData['stateEstimate.pvz'], label=file)


        # plt.show()

    # plt.subplot(plotRows, plotCols, plotCurrent)
    # plt.plot(logData['timestamp'], logData['gyro.x'], '-', label='X')
    # plt.plot(logData['timestamp'], logData['gyro.y'], '-', label='Y')
    # plt.plot(logData['timestamp'], logData['gyro.z'], '-', label='Z')
    # plt.xlabel('timestamp [ms]')
    # plt.ylabel('Gyroscope [°/s]')
    # plt.legend(loc=9, ncol=3, borderaxespad=0.)


if __name__ == '__main__':
    main()