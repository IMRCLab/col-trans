import cfusdlog
import numpy as np
import rowan as rn
import yaml

def main(args=None):
    
    files = ["cf5_06", "cf6_06"]
    att_points = [[0,0.3,0], [0,-0.3,0]]
    shape = 'cuboid'
    logDatas = [cfusdlog.decode(f)['fixedFrequency'] for f in files]

    configData = {}
    configData['robots'] = {}
    configData['payload'] = 'payload.csv'
    if shape == 'point':
        configData['payload_type'] = 'point'
    elif shape == 'triangle':
        configData['payload_type'] = 'triangle'
    elif shape == 'rod':
        configData['payload_type'] = 'rod'
    elif shape == 'cuboid':
        configData['payload_type'] = 'cuboid'
    else:
        print('please add the right shape!')
        exit()
    # payload csv file
    loadPos = np.array([ logDatas[0]['stateEstimateZ.px']/1000,
                         logDatas[0]['stateEstimateZ.py']/1000, 
                         logDatas[0]['stateEstimateZ.pz']/1000,
                         logDatas[0]['stateEstimateZ.vx']/1000,
                         logDatas[0]['stateEstimateZ.vy']/1000, 
                         logDatas[0]['stateEstimateZ.vz']/1000, 
                         logDatas[0]['stateEstimate.pqw'],
                         logDatas[0]['stateEstimate.pqx'],
                         logDatas[0]['stateEstimate.pqy'],
                         logDatas[0]['stateEstimate.pqz']]).T


    with open("output/payload.csv", "w") as f:
            np.savetxt(f,loadPos, delimiter=",")
    
    # cf5
    quatcf5 = rn.from_euler(logDatas[0]['ctrlLeeP.rpyx'], 
                            logDatas[0]['ctrlLeeP.rpyy'], 
                            logDatas[0]['ctrlLeeP.rpyz'], convention='xyz')

    cf5 = np.array([ logDatas[0]['stateEstimateZ.x']/1000,
                         logDatas[0]['stateEstimateZ.y']/1000, 
                         logDatas[0]['stateEstimateZ.z']/1000,
                         logDatas[0]['stateEstimateZ.vx']/1000,
                         logDatas[0]['stateEstimateZ.vy']/1000, 
                         logDatas[0]['stateEstimateZ.vz']/1000,
                         quatcf5[:,0],quatcf5[:,1], quatcf5[:,2], quatcf5[:,3]]).T

    with open("output/cf5.csv", "w") as f:
            np.savetxt(f,cf5, delimiter=",")    

    # cf6
    quatcf6 = rn.from_euler(logDatas[1]['ctrlLeeP.rpyx'], 
                            logDatas[1]['ctrlLeeP.rpyy'], 
                            logDatas[1]['ctrlLeeP.rpyz'], convention='xyz')

    cf6 = np.array([logDatas[1]['stateEstimateZ.x']/1000,
                    logDatas[1]['stateEstimateZ.y']/1000, 
                    logDatas[1]['stateEstimateZ.z']/1000,
                    logDatas[1]['stateEstimateZ.vx']/1000,
                    logDatas[1]['stateEstimateZ.vy']/1000, 
                    logDatas[1]['stateEstimateZ.vz']/1000,
                    quatcf6[:,0],quatcf6[:,1], quatcf6[:,2], quatcf6[:,3]]).T
    
    Fd5 = np.array([
      logDatas[0]['ctrlLeeP.Fdx']  , logDatas[0]['ctrlLeeP.Fdy'], logDatas[0]['ctrlLeeP.Fdz']
    ]).T
    Fd6  = np.array([
      logDatas[1]['ctrlLeeP.Fdx']  , logDatas[1]['ctrlLeeP.Fdy'], logDatas[1]['ctrlLeeP.Fdz']
    ]).T
    
    while np.size(Fd5) != np.size(Fd6):
        if np.size(Fd5) < np.size(Fd6):
            Fd5 =  np.append(Fd5, [Fd5[-1,:]], axis=0)
        elif np.size(Fd5) > np.size(Fd6):
            Fd6 = np.append(Fd6, [Fd6[-1,:]], axis=0)
    
    with open("output/Fd5.csv", "w") as f:
            np.savetxt(f,Fd5, delimiter=",")

    with open("output/Fd6.csv", "w") as f:
        np.savetxt(f,Fd6, delimiter=",")

    while np.size(cf5) != np.size(cf6):
        if np.size(cf5) < np.size(cf6):
            cf5 =  np.append(cf5, [cf5[-1,:]], axis=0)
        elif np.size(cf5) > np.size(cf6):
            cf6 = np.append(cf6, [cf6[-1,:]], axis=0)
        
    with open("output/cf6.csv", "w") as f:
            np.savetxt(f,cf6, delimiter=",")    

    mucf5 = np.array([
      logDatas[0]['ctrlLeeP.desVirtInpx'], logDatas[0]['ctrlLeeP.desVirtInpy'], logDatas[0]['ctrlLeeP.desVirtInpz'] ]).T

    mucf6 = np.array([
      logDatas[1]['ctrlLeeP.desVirtInpx'], logDatas[1]['ctrlLeeP.desVirtInpy'], logDatas[1]['ctrlLeeP.desVirtInpz'] ]).T
    
    size1 = len(logDatas[0]['ctrlLeeP.n1x'])
    cf5hp = np.array([
        logDatas[0]['ctrlLeeP.n1x'], logDatas[0]['ctrlLeeP.n1y'], logDatas[0]['ctrlLeeP.n1z'], np.zeros(size1,) 
    ]).T
    size2 = len(logDatas[1]['ctrlLeeP.n1x'])
    cf6hp = np.array([
        logDatas[1]['ctrlLeeP.n1x'], logDatas[1]['ctrlLeeP.n1y'], logDatas[1]['ctrlLeeP.n1z'], np.zeros(size2,) 
    ]).T

    while np.size(mucf5) != np.size(mucf6):
        if np.size(mucf5) < np.size(mucf6):
            mucf5 =  np.append(mucf5, [mucf5[-1,:]], axis=0)
        elif np.size(mucf5) > np.size(mucf6):
            mucf6 = np.append(mucf6, [mucf6[-1,:]], axis=0)


    while np.size(cf5hp) != np.size(cf6hp):
        if np.size(cf5hp) < np.size(cf6hp):
            cf5hp =  np.append(cf5hp, [cf5hp[-1,:]], axis=0)
        elif np.size(cf5hp) > np.size(cf6hp):
            cf6hp = np.append(cf6hp, [cf6hp[-1,:]], axis=0)
        
    with open('output/mu_cf5.csv', "w") as f:
        np.savetxt(f, mucf5, delimiter=",")


    with open('output/mu_cf6.csv', "w") as f:
        np.savetxt(f, mucf6, delimiter=",")

    with open('output/hp1_cf5.csv', "w") as f:
        np.savetxt(f, cf5hp, delimiter=",")

    with open('output/hp1_cf6.csv', "w") as f:
        np.savetxt(f, cf6hp, delimiter=",")
    
    robots = {
        'cf5': {
        'att': att_points[0],
         'hps': ['hp1_cf5.csv'],
         'mu' : 'mu_cf5.csv',
         'state': 'cf5.csv',
         'Fd': 'Fd5.csv'
        },
        'cf6' : {
        'att': att_points[1],
        'hps': ['hp1_cf6.csv'],
        'mu' : 'mu_cf6.csv',
        'state': 'cf6.csv',
        'Fd': 'Fd6.csv'
    }
    }
    configData['robots'] = robots

    with open("output/configData.yaml", 'w') as f:
            yaml.dump(configData, f)
if __name__ == '__main__':
    main()