"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
from crazyflie_py import Crazyswarm
import numpy as np


Ids = [5, 6]
# TAKEOFF_HEIGHT = 1.3490287
TAKEOFF_HEIGHT = 1.24045172
LAND_HEIGHT    = 0.5
Heights = [0.85778483, 0.85778483]

# offset for the reference trajectories for the two UAVs from payload for 25 degrees
offsets = {
    5: {
        'offset' :  
		#[0.0, 0.360916,   0.77398685]

        [0.0, 0.48983428,  0.69955575]
        #[0.0, 0.2920852,  0.8024974] 20 degrees
        #[0.0, 0.44165386, 0.63074707], #offset for 35 degrees
        # 'offset' : [ 0.0, -0.3169637, 0.67973074],   
                                            
    },
    6: {
        'offset' : 
		#[0.0, -0.1690473,   0.36252311]  
        [0.0,   -0.43247663,  0.61764054]
        #[ 0.0, -0.25788319,  0.70852814]
        #[0.0, -0.40437139, 0.57750219], 
                   #[ 0.0, 0.3169637, 0.67973074],
    }
}

cf_config = {
    5: {
        'waypoints': [
        #        [0.0, 0.360916,   1.17398685]
             [0.0, 0.48983428,  0.99955575]
            #[0.0, 0.2920852,  1.2024974]
            #  [0.0, 0.44165386, 1.09054508]
             #[0.0, 0.48352494,  0.69054508]
             # [0.0, 0.44165386, 1.03074707], 
            #[0.0,  -0.3169637,  1.17973074],  
        ]                                  

    },

    6: {
        'waypoints': [
            #[0.0, -0.1690473,   0.76252311]
             [ 0.0, -0.43247663,  0.91764054]
            #[0.0, -0.25788319,   1.10852814]
            #[0.0, -0.40437139, 1.01272563]
            # [0.0, -0.42903517,  0.61272563]
            #[0.0,  -0.40437139,  0.97750219]
            # [0,  0.3169637,  1.17973074],                             
        ]                                      
    }
}

cf_values = {
    5: {
        'value' : 0.0,
    },
    6: {
        'value' : 1.0,
    }
}


lengths = {
    5: {
        'length' : 0.854 #0.843 #0.77 #0.75, #0.775,
    },
    6: {
        'length' : 0.754 #0.4 #0.754 #0.748 #0.705 #0.75, #0.823,
    }
}

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # allcfs.emergency()

    # for cfid in Ids:
    #     print(cfid, 'take off')
    #     allcfs.crazyfliesById[cfid].takeoff(targetHeight=TAKEOFF_HEIGHT, duration=5.0)
    # timeHelper.sleep(5.5)
    
    # for cfid in Ids:
    #     pos = np.array(allcfs.crazyfliesById[cfid].initialPosition) + np.array([0, 0, TAKEOFF_HEIGHT])
    #     allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)

    # timeHelper.sleep(5.0)

    for cfid in Ids:
        pos = np.array(cf_config[cfid]['waypoints'][0])
        print(cfid, pos)
        allcfs.crazyfliesById[cfid].goTo(pos, 0, 6.0)
    timeHelper.sleep(8.0) 
    
   
#### CHANGE the values of the UAVs and the offset for the setpoints

  
    for cfid in Ids: 
        value = int(cf_values[cfid]['value'])
        offset = offsets[cfid]['offset']
        length = lengths[cfid]['length']
        print('value and ID: ', value, cfid)
        allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.offsetx', offset[0])
        allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.offsety', offset[1])
        allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.offsetz', offset[2])
        #allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.value', value)
        allcfs.crazyfliesById[cfid].setParam('ctrlLeeP.length', length)
    timeHelper.sleep(2.0)

    print('start hovering with QP lee payload')
    

    for cfid in Ids: 
        # allcfs.crazyfliesById[cfid].setParam("usd.logging", 1)
        allcfs.crazyfliesById[cfid].setParam('stabilizer.controller', 7)

    
    timeHelper.sleep(10.0)

###### Linear trajectory on x,y -axes #######################
    # for cfid in Ids:
    #     pos = np.array(cf_config[cfid]['waypoints'][1])
    #     allcfs.crazyfliesById[cfid].goTo(pos, 0, 15.0)
    
    # timeHelper.sleep(17.5)

    # for cfid in Ids:
    #     pos = np.array(cf_config[cfid]['waypoints'][0])
    #     allcfs.crazyfliesById[cfid].goTo(pos, 0, 15.0)
    
    # timeHelper.sleep(17.5)
 ################################################################   
    print('get back to Lee')

    for cfid in Ids: 
        # allcfs.crazyfliesById[cfid].setParam("usd.logging", 0)
        allcfs.crazyfliesById[cfid].setParam('stabilizer.controller', 6)


    for cfid in Ids:
        pos = np.array(allcfs.crazyfliesById[cfid].initialPosition) + np.array([0, 0, LAND_HEIGHT])
        allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)
    timeHelper.sleep(4.0)


    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(3.0)



# TAKEOFF_DURATION = 10.0
# PAYLOAD_CONTROLLER = 30.0
# HOVER_BACK     = 5.0
# TARGET_HEIGHT   = 1.0

    # cf = swarm.allcfs.crazyflies[0]
    # cf.setParam('stabilizer.controller', 6)


    # cf.takeoff(targetHeight=TARGET_HEIGHT, duration=TAKEOFF_DURATION)
    # timeHelper.sleep(TAKEOFF_DURATION)    
    
    
    # cf.setParam("usd.logging", 1)
    # print('start hovering with lee payload')
    # cf.setParam('stabilizer.controller', 7)

    # timeHelper.sleep(PAYLOAD_CONTROLLER)
    
    # # cf.goTo([0.5,0.0,0.0], 0.0, 2.0, relative=True)
    # # timeHelper.sleep(3.0)
    
    # # cf.goTo([-0.5,0.0,0.0], 0.0, 2.0, relative=True)
    # # timeHelper.sleep(3.0)

    # print('finished trajectory')
    # cf.setParam("usd.logging", 0) 

    # print('swap controller')
    # cf.setParam('ctrlLee.mass', 0.034+0.007)
    # cf.setParam('stabilizer.controller', 6)

    # initPos = cf.initialPosition

    # cf.goTo(initPos+[0,0,TARGET_HEIGHT], 0, HOVER_BACK)

    # timeHelper.sleep(HOVER_BACK)
    
    # cf.land(targetHeight=0.04, duration=2.5)
    
    # timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
