import numpy as np
import matplotlib.pyplot as plt
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--filename', type=str, help="numberofcables")
    args   = parser.parse_args()   
    filename = args.filename

    values = open(filename, "r")
    # values = values.read()
    valuesStr = values.read()
    print(valuesStr)

    ValuesList = list(valuesStr.split("\n"))
    cablesNum = []
    runtimeAv = []
    for value in ValuesList:
        value = list(value.split(" "))
        cablesNum.append(float(value[0]))
        runtimeAv.append(float(value[2]))

    p = np.poly1d(np.polyfit(cablesNum, runtimeAv, 2))
    cablesNumSpace = np.linspace(cablesNum[0], cablesNum[-1])
    plt.plot(cablesNum, runtimeAv, 'o', cablesNumSpace, p(cablesNumSpace), '-')
    plt.show()
if __name__ == '__main__':
    main()
