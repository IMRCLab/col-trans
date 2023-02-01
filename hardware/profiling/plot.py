import cfusdlog
import matplotlib.pyplot as plt
import numpy as np
import rowan
# from matplotlib.backends.backend_pdf import PdfPages


def main(args=None):

    files = ["/home/whoenig/tubCloud/projects/coltrans/physical flights/3uav_pm_fig8/cf4_77"]

    logDatas = [cfusdlog.decode(f)['qpSolved'] for f in files]

    for logdata, file in zip(logDatas, files):
        dts = np.diff(logdata['timestamp']) / 1000.0
        freqs = 1/dts
        print("Frequency {:.1f} Hz (stddev: {:.1f})".format(np.mean(freqs), np.std(freqs)))

        fig, axs = plt.subplots(4, 1, sharey=True)

        fd = np.array(logdata['Fd']) / 1000.0 * 8 # convert to ms
        print("Fd {:.1f} ms (stddev: {:.1f})".format(np.mean(fd), np.std(fd)))
        axs[0].hist(fd, bins=30)
        axs[0].set_title("Fd", y=0.7)

        svm = np.array(logdata['svm']) / 1000.0 * 8 # convert to ms
        print("SVM {:.1f} ms (stddev: {:.1f})".format(np.mean(svm), np.std(svm)))
        axs[1].hist(svm, bins=30)
        axs[1].set_title("Svm", y=0.7)

        mu = np.array(logdata['mu']) / 1000.0 * 8 # convert to ms
        print("Mu {:.1f} ms (stddev: {:.1f})".format(np.mean(mu), np.std(mu)))
        axs[2].hist(mu, bins=30)
        axs[2].set_title("Mu", y=0.7)

        total = np.array(logdata['total']) / 1000.0 * 8 # convert to ms
        print("Total {:.1f} ms (stddev: {:.1f})".format(np.mean(total), np.std(total)))
        axs[3].hist(total, bins=30)
        axs[3].set_title("Total", y=0.7)

        axs[-1].set_xlabel("Runtime [ms]")


        plt.show()


if __name__ == '__main__':
    main()