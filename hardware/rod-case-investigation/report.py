from helper import *
import argparse

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="config yaml file, name only without extension (assumed to be .yaml)")
    parser.add_argument("--output", default=None,  type=str, help="save output file, name only without extension (assumed to be .yaml)")
    parser.add_argument("--pdf", default="flight",  type=str, help="save output file, name only without extension (assumed to be .pdf)")
    parser.add_argument("--start", default=0.0,  type=float, help="start time")
    parser.add_argument("--end", default=40.0,  type=float, help="end time")
    
    args = parser.parse_args()
    data = loadyaml(args.config + ".yaml")
    flights = data["flights"]["data"]
    name = data["flights"]["name"]
    extract_data = data["flights"]["extract"]
    plot_enabled = data["plot"]["enabled"]
    if plot_enabled and "files" in data["plot"].keys():
        plot_files = data["plot"]["files"]
    else:
        plot_files = []
    start_time = args.start
    end_time = args.end
    flightsData = []
    starttimes = []
    
    for files in flights:
        starttime, logDatas = extractData(files, start_time=start_time, end_time=end_time)
        flightsData.append(logDatas)
        starttimes.append(starttime)
    data_to_plot = dict()
    
    if extract_data:
        for flight_num, logDatas in enumerate(flightsData):
            data_to_plot[flight_num] = dict()
            for page_num, page_value in data["data_to_plot"].items():
                len_data = int(len(page_value["data"].keys())/2)
                data_to_plot[flight_num][page_num] = dict()

                for k, filename in enumerate(flights[flight_num]):
                    data_to_plot[flight_num][page_num][filename] = dict()
                    data_to_plot[flight_num][page_num][filename]["name_data"] = dict()
                    data_to_plot[flight_num][page_num][filename]["title"] = page_value["title"]
                    data_to_plot[flight_num][page_num][filename]["num_of_plots"] = page_value["num_of_plots"]
                    data_to_plot[flight_num][page_num][filename]["plot_labels"] = page_value["plot_labels"]
                    data_to_plot[flight_num][page_num][filename]["special"] = page_value["special"]
                    data_to_plot[flight_num][page_num][filename]["time"] = ((logDatas[k]['timestamp'] - starttimes[flight_num])/1000.0).tolist()
                    data_to_plot[flight_num][page_num][filename]["axis_name"] = []

                    for i in range(len_data):
                        data_name = page_value["data"][f"data{i+1}"] # data from the usd in the config
                        axis_name = page_value["data"][f"name{i+1}"] # namei in the config
                        unit = page_value["unit"]
                        special = page_value["special"]
                        data_to_plot[flight_num][page_num][filename]["axis_name"].extend([axis_name])
                        data_to_plot[flight_num][page_num][filename]["name_data"][f"name{i+1}"] = []
                        data_to_plot[flight_num][page_num][filename]["name_data"][f"data{i+1}"] = dict()
                        
                        for data_i in data_name: 
                            # data_i_name = data_i.split(".",1)[1]
                            data_i_name = data_i
                            data_to_plot[flight_num][page_num][filename]["name_data"][f"name{i+1}"].append(data_i_name)
                            data_to_plot[flight_num][page_num][filename]["name_data"][f"data{i+1}"][data_i_name] = getData(logDatas[k], [data_i], unit)
                        # compute special computations: e.g., motor forces from thrustPart, pitchPart, rollPart, yawPart
                        # Note that: the final data_to_plot needs to be updated accordingly in the helper script. 
                        if special:
                            if axis_name == "Thrust":
                                motorForces = computeMotorForces(data_to_plot[flight_num][page_num][filename]["name_data"], i)
                                data_to_plot[flight_num][page_num][filename]["name_data"] = motorForces
        saveyaml(args.output + "_config",data_to_plot)
    

    else: 
        data_to_plot = loadyaml(args.output + "_config.yaml")
    

    # generate plots
    if plot_enabled:
        for flight_num, cfs in enumerate(plot_files):
            flight_value = data_to_plot[flight_num]
            f = PdfPages(f'{args.pdf}_results_{flight_num}.pdf')
            for cf in cfs:
                for page_key, page_value in flight_value.items():
                    if page_key == "time":
                        break
                    # create and add fig to pdf
                    fig = create_fig(page_value[cf], cf)
                    fig.savefig(f, format='pdf', bbox_inches='tight')
            f.close()



    stats_dict = computeStats(data_to_plot, flights)
    saveyaml2(args.output, stats_dict)

    
if __name__=="__main__":
    main()