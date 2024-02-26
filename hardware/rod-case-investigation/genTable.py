import numpy as np
from pathlib import Path
from helper import *
import shutil
import subprocess
import multiprocessing as mp
import tqdm
import psutil
import os
import sys


def prepareConfig(extract_bool, files_to_plot, plot_bool, start_time, end_time):
    config = dict()
    config["flights"] = dict()
    config["plot"] = dict()
    config["flights"]["extract"] = extract_bool
    config["start_time"] = start_time
    config["end_time"] = end_time
    config["plot"]["enabled"] = plot_bool
    return config



def run_script(config):
    path = config[1]
    configs_path = path["configs"]
    output_path = path["output"]
    log_path = path["log"]
    pdf_path = path["pdf"]
    flight_name = config[0]["flights"]["name"]
    start_time = config[2]
    end_time = config[3]

    with open(log_path /  f"{flight_name}_output_log.txt", 'w') as f:
        subprocess.run(["python3",
            "report.py", 
            "--config", configs_path / flight_name,
            "--output", output_path / flight_name,
            "--pdf", pdf_path / flight_name,
            "--start", start_time,
            "--end", end_time
            ],
            stdout=f, stderr=f, check=True)    


def writeTable():
    pass

def main():

    parallel = True
    config_table = loadyaml("config_table.yaml")

    extract_bool = config_table["flights"]["extract"]
    plot_bool = config_table["plot"]["enabled"]
    data_to_extract = config_table["flights"]["data"]
    files_to_plot = config_table["plot"]["files"]
    pages_to_plot = config_table["data_to_plot"]
    start_time = config_table["start_time"]
    end_time = config_table["end_time"]

    table = config_table["table"]
    max_cpus = config_table["max_cpus"]

    configs = []
    paths = dict()
    result_path = Path("result/")
    configs_path = result_path / Path("configs/")
    log_path = result_path / Path("logs/")
    pdf_path = result_path / Path("pdf/")
    output_path = result_path / Path("output/")

    paths["configs"]  = configs_path
    paths["result"]  = result_path
    paths["log"]  = log_path
    paths["output"]  = output_path
    paths["pdf"]  = pdf_path

    if config_table["load_files"]:
        if result_path.exists():
            shutil.rmtree(result_path)
        result_path.mkdir(parents=True, exist_ok=False)

        if configs_path.exists():
            shutil.rmtree(configs_path)
        configs_path.mkdir(parents=True, exist_ok=False)
        
        if log_path.exists():
            shutil.rmtree(log_path)
        log_path.mkdir(parents=True, exist_ok=False)


        if output_path.exists():
            shutil.rmtree(output_path)
        output_path.mkdir(parents=True, exist_ok=False)


        if pdf_path.exists():
            shutil.rmtree(pdf_path)
        pdf_path.mkdir(parents=True, exist_ok=False)

        flights_iter = 0
        for data, data_to_extract in data_to_extract.items():
            for flight, data_to_analyze in data_to_extract.items():
                if len(data_to_analyze) > 0: 
                    config = dict()
                    config["flights"] = dict()
                    config["plot"] = dict()
                    config["flights"]["extract"] = extract_bool
                    start_time_i = start_time[flights_iter]
                    end_time_i   = end_time[flights_iter]
                    config["plot"]["enabled"] = plot_bool

                    config["flights"]["data"] =  data_to_analyze
                    config["flights"]["name"] = flight + "_" + data
                    config["flights"]["extract"] = extract_bool
                    for file,f_to_extract in files_to_plot.items():
                        if len(f_to_extract) > 0: 
                            for f, f_data in f_to_extract.items():
                                if len(f_data) > 0 and config["flights"]["name"] == f+"_"+file:
                                    filename = f +"_"+file
                                    config["plot"]["files"] = f_data
                        config["data_to_plot"] = pages_to_plot
                    config_path = configs_path / config["flights"]["name"]
                    saveyaml(str(config_path), config)
                    configs.append((config, paths, str(start_time_i), str(end_time_i)))
            flights_iter+=1

        max_cpus = 3 # limit the number of CPUs due to high memory usage
        if parallel: 
            use_cpus = min(max_cpus, psutil.cpu_count(logical=False)-1)
            print("Using {} CPUs".format(use_cpus))
            with mp.Pool(use_cpus) as p:
                for _ in tqdm.tqdm(p.imap_unordered(run_script, configs)):
                    pass
    # Load files
    files_yaml = os.listdir(output_path)
    robots = table["num_robots"]
    envs = table["envs"]
    metrics = table["metrics"]
    algs = table["algs"]
    table_path = Path(Path(os.getcwd()+"/result/") / table["name"])

    # instances = ["{}cfs_{}_{}".format(n, env, alg) for env in envs for alg in algs for n in robots]
    # instances.remove("3cfs_forest_geom")
    
    with open(table_path.with_suffix(".tex"), "w") as f:
        f.write(r"\documentclass{standalone}")
        f.write("\n")
        f.write(r"\usepackage{xcolor}")
        f.write(r"\usepackage{multirow}")
        f.write("\n")
        f.write(r"\begin{document}")
        f.write("\n")
        f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")
        columns = "{" 
        for i in range(len(robots)+3):
            if i == len(robots)+2:
                columns+="l}"
            else:
                columns+="l|"
        f.write(r"\begin{tabular}" + columns + "\n")
        f.write("Environment & Metrics &"+ r" & ".join([str(n)  + " robots" for n in robots]) + r" & success [\%]\\" + "\n")

        rows_num = str(len(metrics) * 2)
        for env in envs: 
            f.write(r"\hline" + "\n")
            f.write(r"\multirow{"+rows_num+r"}{*}{" + env.capitalize() + r"}")
            #tracking error
            for k, alg in enumerate(algs):
                out = " & Error " + alg + " [m]"
                success_rate = 0
                success_count = 0
                for n in robots:
                    inst = "{}cfs_{}_{}.yaml".format(n, env, alg)
                    out += " & "
                    if inst in files_yaml:
                        inst_values = loadyaml("result/output/"+inst)
                        ep_mean = inst_values["ep_mean"]["mean"]
                        ep_std = inst_values["ep_mean"]["std"]
                        if alg == "opt":
                            out +=" {{ \\textbf{{{:.2f}}}}}  ".format(ep_mean)
                        else: 
                            out +=" {{ {:.2f}}}  ".format(ep_mean)
                        out += " {{\\color{{gray}}\\tiny {:.2f} }} ".format(ep_std)
                        out += " {\\tiny " + str(inst_values["trials"]*10) + " \\% } "
                        success_rate +=  inst_values["trials"]*10
                    else: 
                        out+=" \hspace{0.1cm}--- "
                    success_count += 1
                out += "& {:.0f} ".format(success_rate/success_count) 
                out += r"\\"
                f.write(out)

            f.write(r"\cline{2-4}" + "\n")
            # Energy
            for alg in algs:
                out = " & Energy " + alg + " [Wh] "
                for n in robots:
                    inst = "{}cfs_{}_{}.yaml".format(n, env, alg)
                    out += " & "
                    if inst in files_yaml:
                        inst_values = loadyaml("result/output/"+inst)
                        energy_mean = inst_values["energy_mean"]
                        energy_std = inst_values["energy_std"]
                        if alg == "opt":
                            out +=" {{ \\textbf{{{:.2f}}}}}  ".format(energy_mean)
                        else: 
                            out +=" {{ {:.2f}}}  ".format(energy_mean)
                        out += " {{\\color{{gray}}\\tiny {:.2f} }} ".format(energy_std)
                    else: 
                        out+=" \hspace{0.1cm}--- "
                out += r"\\"
                f.write(out)

        f.write("\n")
        f.write(r"\end{tabular}")
        f.write("\n")
        f.write(r"\end{document}")

        
    gen_pdf(table_path)
    #         pass
    #     pass

if __name__ == "__main__":
    main()