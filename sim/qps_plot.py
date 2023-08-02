import matplotlib.pyplot as plt
import numpy as np
import subprocess
from pathlib import Path
import yaml
from multiprocessing import Pool

def extract_number(filename):
    # Extract the number between "logTime_" and "_pm.yaml"
    return int(filename.split("_")[1])

def run_sim(cfg):
    # simulate for all the cfgs
    try:
        result = subprocess.run(
            ["python3",
            "simulate.py",
            cfg])  # Use check=True to raise exception on non-zero exit status
    except Exception as e:
        print(e)



def create_whisker_plot(data, pdfname="result.pdf", title='Whisker Plot', ylabel='Values', xlabel=[2]):
    mv = []
    for d in data:
        mean = np.mean(d)
        std = np.std(d)
        mv.append((mean, std))
    print(pdfname, '\n', mv)
    plt.figure()
    plt.boxplot(data, labels=xlabel, patch_artist=True, boxprops=dict(facecolor='lightblue'), showfliers=True)
    plt.title(title)
    plt.xlabel("Number of UAVs")
    plt.ylabel(ylabel)
    # plt.yscale("log")
    plt.grid()
    # plt.savefig(pdfname)  # Save the plot as a PDF file
    plt.show()

def main():
    payloadType = "rig"
    # payloadType = "pm"
    cfg_path = Path("cfg/{}".format(payloadType))
    cfgnames = []
    
    # Iterate through  items (files with yaml extension) in the folder
    for item in cfg_path.glob("*.yaml"):
        # Check if the item is a file (not a directory)
        if item.is_file():
            # Add the filename to the list
            cfgnames.append(cfg_path/item.name)

    # with Pool(processes=3) as pool:
    #     for i in pool.imap_unordered(run_sim, cfgnames):
    #         pass

    result_path = Path("output")
    timeLogs = []
    data = []
    self_total_time_data = []
    total_time_n_data = []
    for item in result_path.glob("logTime_*_{}.yaml".format(payloadType)):
        if item.is_file():
            timeLogs.append(result_path.name+"/"+item.name)
    timeLogs = sorted(timeLogs, key=extract_number)
    uav_nums = []
    for log in timeLogs:
        print(log)
        with open(log, "r") as f:
            data_i = yaml.load(f, Loader=yaml.FullLoader)
        uav_num = data_i["uavs_num"]
        uav_nums.append(uav_num)
        payloadType = data_i["type"]
        uavs = list(data_i["robots"].keys())
        
        if payloadType == "rig":
            time_svm         = [time[0] for time in data_i["robots"][uavs[1]]]
            time_mu          = [time[1] for time in data_i["robots"][uavs[1]]]
            time_Fd          = [time[2] for time in data_i["robots"][uavs[1]]]
            time_total_self  = [time[3] for time in data_i["robots"][uavs[1]]]
            time_total = [svm +  Fd  + mu for svm, mu, Fd in zip(time_svm, time_mu, time_Fd)]
            print(uav_num)
            time_total_n = [svm * uav_num +  Fd * uav_num + mu for svm, mu, Fd in zip(time_svm, time_mu, time_Fd)]
             
        else:
            uav_num = data_i["uavs_num"]
            time_svm         = [time[0] for time in data_i["robots"][uavs[1]]]
            time_mu          = [time[1] for time in data_i["robots"][uavs[1]]]
            time_total_self  = [time[2] for time in data_i["robots"][uavs[1]]]
            time_total = [svm * uav_num + mu for svm, mu in zip(time_svm, time_mu)]
            time_total_n = [svm * uav_num +  Fd * uav_num + mu for svm, mu, Fd in zip(time_svm, time_mu, time_Fd)]

        data.append(time_total)
        self_total_time_data.append(time_total_self)
        total_time_n_data.append(time_total_n)
    
    # Call the function to create the whisker plots for each UAV's dataset
    create_whisker_plot(data, pdfname="whisker_plot_rig_ms_dis.pdf", title='Whisker Plots for Different UAVs', ylabel='time [ms]', xlabel=[i for i in range(min(uav_nums),max(uav_nums)+1)])
    create_whisker_plot(self_total_time_data, pdfname="whisker_plot_rig_ms_total_time.pdf", title='Whisker Plots for Different UAVs', ylabel='time [ms]', xlabel=[i for i in range(min(uav_nums),max(uav_nums)+1)])
    create_whisker_plot(total_time_n_data, pdfname="whisker_plot_rig_ms_cen_n.pdf", title='Whisker Plots for Different UAVs', ylabel='time [ms]', xlabel=[i for i in range(min(uav_nums),max(uav_nums)+1)])
    
if __name__ == "__main__":
    main()