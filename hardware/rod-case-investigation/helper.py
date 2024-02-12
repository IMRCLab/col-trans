import cfusdlog
import yaml
import numpy as np

from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.gridspec import SubplotSpec, GridSpec
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from mpl_toolkits import mplot3d 
plt.rcParams['axes.grid'] = True
plt.rcParams['figure.max_open_warning'] = 100



def create_subtitle(fig: plt.Figure, grid: SubplotSpec, title: str):
    row = fig.add_subplot(grid)
    row.set_title('\n\n\n'+title, fontweight='medium',fontsize='medium')
    row.set_frame_on(False)
    row.axis('off')


def create_fig(cf_data, cf_name):
    num_of_plots = cf_data["num_of_plots"]
    title = str(cf_data["title"]) + " " + cf_name
    time = cf_data["time"]
    axis_name = cf_data["axis_name"]
    plot_labels = cf_data["plot_labels"]
    len_data = int(len(cf_data["name_data"].keys())/2)
    names = []
    datas = []
    for i in range(len_data):
        names.append(cf_data["name_data"][f"name{i+1}"])
        datas.append(cf_data["name_data"][f"data{i+1}"])
    fig, ax = plt.subplots(num_of_plots, 1, sharex=True)
    fig.tight_layout()
    for i, data in enumerate(datas):
        name = names[i]
        for j, axis in enumerate(name):
            # add special conditions for special data
            if axis_name[0] == "Thrust":
                if j < 4: 
                    # special plot for the thrust to add maxThrust
                    ax[j].plot(time, data["maxThrust"], lw=0.75,label="maxThrust")
                    ax[j].plot(time, data[axis], lw=0.75, label=f"{axis}")
                    ax[j].set_ylabel(plot_labels[j])
                    ax[j].legend()
            else:
                ax[j].plot(time, data[axis], lw=0.75, label=f"{axis}")
                ax[j].set_ylabel(plot_labels[j])
                ax[0].legend()
        grid = plt.GridSpec(num_of_plots, 1)
        create_subtitle(fig, grid[0, ::], title)
    fig.supxlabel("time [s]",fontsize='small')
    return fig


def loadyaml(file_in):
    with open(file_in, "r") as f:
        file_out = yaml.load(f,Loader=yaml.CSafeLoader)
    return file_out
        

def saveyaml(file_dir, data):
    with open(file_dir + '.yaml', 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=None)

def saveyaml2(file_dir, data):
    with open(file_dir + '.yaml', 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False)

def flatten(xss):
    return [x for xs in xss for x in xs]

def getData(logDatas, dataname, unit):
    out = []
    for data in dataname: 
        res = np.array(logDatas[data])
        if unit == "mm":
            res = res / 1000.0
        elif unit == "g":
            res = res / 100.0
        out.extend(res.tolist())
    return out


def extractData(files, start_time=0, end_time=100):
    logDatas = [cfusdlog.decode(f)['fixedFrequency'] for f in files]
    starttime = min([logDatas[k]['timestamp'][0] for k in range(len(files))])
    # filter by time
    for k in range(len(logDatas)):
        t = (logDatas[k]['timestamp'] - starttime)/1000
        idx = np.where(np.logical_and(t > start_time, t < end_time))
        for key, value in logDatas[k].items():
            logDatas[k][key] = value[idx]
    return starttime, logDatas


def computeStats(data, flights):
    ep_trial = []
    energy_trials = []
    trials = 0
    for flight_num, cfs in enumerate(flights):
        flight_data = data[flight_num]
        motorForces = []
        for i, cf in enumerate(cfs):
            for page_key, page_value in flight_data.items():
                cf_data = page_value[cf]
                if cf_data["title"] =="Payload Pose":
                    data_p0 = np.array(list(cf_data["name_data"]["data1"].values())) 
                    data_p0d = np.array(list(cf_data["name_data"]["data2"].values())) 
                    ep_trial.extend(np.linalg.norm(data_p0-data_p0d, axis=0)*100) 
                elif cf_data["title"] == "Thrust":
                    motor_data = np.array(list(cf_data["name_data"]["data1"].values()))
                    motor_thrust = motor_data[0:4, :]
                    motorForces.append(motor_thrust)
    
        min_size = min(arr.shape[1] for arr in motorForces)
        motorForces_trimmed =  [arr[:, :min_size] for arr in motorForces]

        motorForces_stack = np.concatenate(motorForces_trimmed, axis=0)
        force = np.sum(motorForces_stack, axis=0)/4
        power = force / 4
 
        energy = np.sum(power.tolist())*0.01/60/60 # Wh
        energy_trials.append(energy)
        trials += 1
    
    stats_dict = dict()
    stats_dict["energy_mean"] = np.mean(energy_trials).tolist()
    stats_dict["energy_std"] = np.std(energy_trials).tolist()
    stats_dict["energy_unit"] = "Wh"
    stats_dict["trials"] = trials
    stats_dict["ep_mean"] = dict()
    # stats_dict["ep_mean"]["mean"] = dict()
    # stats_dict["ep_mean"]["std"] = dict()
    stats_dict["ep_mean"]["unit"] = "cm"
    flights_flattened = flatten(flights)
    
    # for cf, ep in zip(flights_flattened, ep_trial):
    stats_dict["ep_mean"]["mean"] = float(np.mean(ep_trial))
    stats_dict["ep_mean"]["std"] = float(np.std(ep_trial))

    return stats_dict

## Special computations are added here:
def computeMotorForces(motor_components, i):
    names =  motor_components[f"name{i+1}"]
    motorpart = []
    for name in names: 
        motorpart.append(np.array([motor_components[f"data{i+1}"][name]]))                   

    motor_components[f"name{i+1}"] = dict()
    motor_components[f"name{i+1}"] = ["f1", "f2", "f3", "f4", "maxThrust"]
    motor_components[f"data{i+1}"] = dict()

    motor_components[f"data{i+1}"]["f1"] = np.array(motorpart[0] - motorpart[1] - motorpart[2] + motorpart[3])[0].tolist()
    motor_components[f"data{i+1}"]["f2"] = np.array(motorpart[0] - motorpart[1] + motorpart[2] - motorpart[3])[0].tolist()
    motor_components[f"data{i+1}"]["f3"] = np.array(motorpart[0] + motorpart[1] + motorpart[2] + motorpart[3])[0].tolist()
    motor_components[f"data{i+1}"]["f4"] = np.array(motorpart[0] + motorpart[1] - motorpart[2] - motorpart[3])[0].tolist()
    motor_components[f"data{i+1}"]["maxThrust"] = np.array(motorpart[4])[0].tolist()
    
    return motor_components

def main():
    out = loadyaml("config.yaml")
    print(out)

if __name__=="__main__":
    main()