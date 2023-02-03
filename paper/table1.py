import cfusdlog
# import matplotlib.pyplot as plt
import numpy as np
import rowan
import yaml
from pathlib import Path
# from matplotlib.backends.backend_pdf import PdfPages


def main(args=None):

    cloudfolder = "/home/whoenig/tubCloud/projects/coltrans/physical flights/"

    with open("table1.yaml", 'r') as ymlfile:
        cfg = yaml.safe_load(ymlfile)


    for trial in cfg["cases"]:

        files = trial["files"]

        # compute starttime (over all events!)
        logDatas = [cfusdlog.decode(Path(cloudfolder) / f) for f in files]

        starttimes = []
        for k in range(len(logDatas)):
            for event, data in logDatas[k].items():
                starttimes.append(min(data['timestamp']))
        starttime = min(starttimes)

        # filter by time
        for k in range(len(logDatas)):
            for event, data in logDatas[k].items():
                t = (data['timestamp'] - starttime)/1000
                idx = np.where(np.logical_and(t > trial["start_time"], t < trial["end_time"]))
                for key, value in data.items():
                    data[key] = value[idx]


        trial["freqs"] =[]
        trial["fds"] = []
        trial["svms"] = []
        trial["mus"] = []
        trial["totals"] = []

        for k in range(len(logDatas)):
            logdata = logDatas[k]["qpSolved"]

            # Profiling information
            dts = np.diff(logdata['timestamp']) / 1000.0
            freqs = 1/dts
            trial["freqs"].extend(freqs.tolist())

            fd = np.array(logdata['Fd']) / 1000.0 * 8 # convert to ms
            trial["fds"].extend(fd.tolist())

            svm = np.array(logdata['svm']) / 1000.0 * 8 # convert to ms
            trial["svms"].extend(svm.tolist())

            mu = np.array(logdata['mu']) / 1000.0 * 8 # convert to ms
            trial["mus"].extend(mu.tolist())
            
            total = np.array(logdata['total']) / 1000.0 * 8 # convert to ms
            trial["totals"].extend(total.tolist())
        
        trial["pos_errors"] = []
        trial["rot_errors"] = []

        for k in range(len(logDatas)):
            logdata = logDatas[k]["fixedFrequency"]

            # Tracking error

            p0 = np.array([logdata['stateEstimateZ.px']/1000,
                                logdata['stateEstimateZ.py']/1000, 
                                logdata['stateEstimateZ.pz']/1000,
                                ]).T

            p0d = np.array([logdata['ctrltargetZ.x']/1000,
                                    logdata['ctrltargetZ.y']/1000, 
                                    logdata['ctrltargetZ.z']/1000,
                                    ]).T

            error = p0 - p0d
            eucl_error = np.linalg.norm(error, axis=1)*100 # in cm
            trial["pos_errors"].extend(eucl_error.tolist())


            q = np.array([ 
                logdata['stateEstimate.pqw'],
                logdata['stateEstimate.pqx'],
                logdata['stateEstimate.pqy'],
                logdata['stateEstimate.pqz']]).T

            qp_des = np.array([ 
                logdata['ctrlLeeP.qp_desw'],
                logdata['ctrlLeeP.qp_desx'],
                logdata['ctrlLeeP.qp_desy'],
                logdata['ctrlLeeP.qp_desz']]).T

            error_quat = np.degrees(rowan.geometry.intrinsic_distance(q, qp_des)) # in deg
            trial["rot_errors"].extend(error_quat.tolist())

    # print(cfg)

    print(r"%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    print(r"% Generated table, do not edit!")
    print(r"\begin{tabular}{l||c||c||c||c||c}")
    print(r"& " + " & ".join([trial["name"] for trial in cfg["cases"]]) + r"\\")
    
    print(r"\hline\hline")

    print(r"Payload & ", end='')
    print(" & ".join(["{}".format(trial["payload"]) for trial in cfg["cases"]]) + r"\\")
    print(r"Trajectory & ", end='')
    print(" & ".join(["{}".format(trial["trajectory"]) for trial in cfg["cases"]]) + r"\\")
    print(r"Mass [g] & ", end='')
    print(" & ".join(["{}".format(trial["mass"]) for trial in cfg["cases"]]) + r"\\")
    print(r"Dimension [cm] & ", end='')
    print(" & ".join(["{}".format(trial["dimensions"]) for trial in cfg["cases"]]) + r"\\")
    print(r"Cables [cm] & ", end='')
    print(" & ".join([", ".join(str(l) for l in trial["cables"]) for trial in cfg["cases"]]) + r"\\")
    
    print(r"\hline")

    print(r"QP runtime total [ms] & ", end='')
    print(" & ".join(["{:.1f} ({:.1f})".format(np.mean(trial["totals"]), np.std(trial["totals"])) for trial in cfg["cases"]]) + r"\\")

    print(r"\quad QP runtime Fd [ms] & ", end='')
    print(" & ".join(["{:.1f} ({:.1f})".format(np.mean(trial["fds"]), np.std(trial["fds"])) for trial in cfg["cases"]]) + r"\\")

    print(r"\quad QP runtime SVM [ms] & ", end='')
    print(" & ".join(["{:.1f} ({:.1f})".format(np.mean(trial["svms"]), np.std(trial["svms"])) for trial in cfg["cases"]]) + r"\\")

    print(r"\quad QP runtime $\boldsymbol{\mu}$ [ms] & ", end='')
    print(" & ".join(["{:.1f} ({:.1f})".format(np.mean(trial["mus"]), np.std(trial["mus"])) for trial in cfg["cases"]]) + r"\\")

    print(r"\hline")

    print(r"Payload position error [cm] & ", end='')
    print(" & ".join(["{:.1f} ({:.1f})".format(np.mean(trial["pos_errors"]), np.std(trial["pos_errors"])) for trial in cfg["cases"]]) + r"\\")
    print(r"Payload rotation error [deg] & ", end='')
    print(" & ".join(["{:.1f} ({:.1f})".format(np.mean(trial["rot_errors"]), np.std(trial["rot_errors"])) if np.isfinite(trial["rot_errors"]).all() else " - " for trial in cfg["cases"]]) + r"\\")


    print(r"\end{tabular}")
    print(r"%%%%%%%%%%%%%%%%%%%%%%%%%%%")


if __name__ == '__main__':
    main()