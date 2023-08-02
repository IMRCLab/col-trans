import argparse
import numpy as np
import time
import rowan as rn
import yaml
# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
np.set_printoptions(suppress=True)
import pathlib
currPath = pathlib.Path(__file__).parent.resolve()

def normVec(n):
    return n / np.linalg.norm(n)

# copied from https://github.com/rdeits/meshcat-python/blob/master/src/meshcat/geometry.py#L83
# since the latest pip-version doesn't include it yet


class Plane(g.Geometry):

    def __init__(self, material=g.MeshPhongMaterial(), width=1, height=1, widthSegments=1, heightSegments=1):
        super(Plane, self).__init__()
        self.width = width
        self.height = height
        self.widthSegments = widthSegments
        self.heightSegments = heightSegments
        self.material = material

    def lower(self, object_data):
        return {
            u"uuid": self.uuid,
            u"type": u"PlaneGeometry",
            u"width": self.width,
            u"height": self.height,
            u"widthSegments": self.widthSegments,
            u"heightSegments": self.heightSegments,
        }


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--uavs', type=int, help="num of uavs")
    parser.add_argument('--load', type=str, help="load type: pm or rig")
    args   = parser.parse_args()   
    
    
    with open(str(currPath)+'/cfg/vis.yaml') as f:
        meshcatdata = yaml.load(f, Loader=yaml.FullLoader)


    with open(str(currPath)+'/output_{}_{}/configData.yaml'.format(args.uavs, args.load)) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        
    uavs         = data['robots']

    payload      = data['payload']
    
    visProps     = meshcatdata['meshcat']
    robotshape   = str(currPath) + '/' + meshcatdata['robot']['shape']
    colors       = meshcatdata['robot']['colors']
    hpShape      = meshcatdata['robot']['hps']
    muShape      = meshcatdata['robot']['mu']
    cableShape   = meshcatdata['robot']['cable']
    constrShape  = meshcatdata['robot']['constrSphere']
    ploadShape   = data['payload_type']

    vis = meshcat.Visualizer()
    if visProps['openWindow'] == True:
        vis.open()

    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-30), np.radians(90))))

    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([1, 0, 0]))
    # data 
    states = {}
    hps    = {}
    mus    = {}
    Fds    = {}

    # shapes
    Quadspheres  = {}
    constSpheres = {}
    hplanes      = {}
    normals      = {}
    cables       = {}
    mushape      = {}
    Fdshape      = {}
    quadNums     = len(uavs.keys())
    for id, uav, qNum in zip(uavs.keys(), uavs.values(), range(quadNums)):
        # File paths for states
        stateFilePath = uav['state']
        hpsFilePaths  = uav['hps']
        musFilePath   = uav['mu']
        FdFilePath    = uav['Fd']

        
        # Add states for uav
        states[id]    = np.loadtxt(str(currPath)+'/output_{}_{}/'.format(args.uavs, args.load) + stateFilePath, delimiter=',')
        Fds[id]       = np.loadtxt(str(currPath)+'/output_{}_{}/'.format(args.uavs, args.load) + FdFilePath, delimiter=',')
        # Add shape for uav and its constraint sphere
        
        Quadspheres[id]    = g.StlMeshGeometry.from_file(robotshape)
        constSpheres[id]   = g.Mesh(g.Sphere(constrShape['radius'][qNum]), 
                            material=g.MeshLambertMaterial(opacity=constrShape['opacity'], color=constrShape['color']))
        # Add states and materials for hps        
        hptmp  = {} # tmp dict
        hplanetmp      = {}
        normaltmp      = {}
        for i, hpFilePath in zip(range(len(hpsFilePaths)), hpsFilePaths):
            # Add states for hyperplane
            hptmp[i]       = np.loadtxt(str(currPath)+'/output_{}_{}/'.format(args.uavs, args.load) + hpFilePath, delimiter=',')
            hps[id]        = hptmp.copy()
            # Add materials for hyperplanes and normals
            hpcolor        = colors[qNum]
            hpopacity      = hpShape['opacity']
            size           = hpShape['size']
            linewidthNorm  = hpShape['linewidth']
            
            normaltmp[i]   = g.LineBasicMaterial(linewidth=linewidthNorm, color=hpcolor, opacity=hpopacity)
            hplanetmp[i]   =  (Plane(width=size[0], height=size[1]), 
                              g.MeshBasicMaterial(opacity=hpopacity, color=hpcolor))
            hplanes[id]    = hplanetmp 
            normals[id]    = normaltmp
        ##
        # Add state for mu vector
        mus[id]            = np.loadtxt(str(currPath)+'/output_{}_{}/'.format(args.uavs, args.load) + musFilePath, delimiter=',')
        # Add shape for mu vector 
        mushape[id]        =  g.LineBasicMaterial(linewidth=muShape['linewidth'], color=colors[qNum])
        Fdshape[id]        =  g.LineBasicMaterial(linewidth=muShape['linewidth'], color=colors[qNum])
        # Add shape for cable 
        cables[id]         = g.LineBasicMaterial(linewidth=cableShape['linewidth'], color=cableShape['color'])
    
    plstatePath = payload
    plstates     = np.loadtxt(str(currPath)+'/output_{}_{}/'.format(args.uavs, args.load) + plstatePath, delimiter=',')
    if ploadShape == 'triangle':
        loadshape   = meshcatdata['triangle']
        vis["payload"].set_object(g.StlMeshGeometry.from_file(loadshape['shape']), g.MeshLambertMaterial(color=loadshape['color']))
    elif ploadShape == 'point':
        loadshape   = meshcatdata['sphere']
        vis["payload"].set_object(g.Mesh(g.Sphere(loadshape['radius']), g.MeshLambertMaterial(color=loadshape['color'])))
    elif ploadShape == 'rod':
         loadshape   = meshcatdata['cuboid']
         vis["payload"].set_object(g.Mesh(g.Box(loadshape['size']),
         g.MeshLambertMaterial(color=loadshape['color'])))
    elif ploadShape == 'cuboid':
         loadshape   = meshcatdata['cuboid']
         vis["payload"].set_object(g.Mesh(g.Box(loadshape['size']),
         g.MeshLambertMaterial(color=loadshape['color'])))

    if visProps["dt"] > 0:
        steps = int(1/visProps["dt"])
        plstates = plstates[::steps,:]
        for id in uavs.keys():
            states[id] = states[id][::steps, :]
            Fds[id] = Fds[id][::steps,:]
            mus[id] = mus[id][::steps,:]
            if len(list(uavs.keys())) > 1:
                for hp in hps[id]:
                    hps[id][hp] = hps[id][hp][::steps,:]

    while True:    
        tick = 0
        for plstate in plstates:
            ppos = plstate[0:3]
            if ploadShape == 'triangle' or ploadShape == 'rod' or ploadShape == 'cuboid':
                vis["payload"].set_transform(
                            tf.translation_matrix(ppos).dot(
                tf.quaternion_matrix([plstate[6],plstate[7],plstate[8],plstate[9]])))
            elif ploadShape == 'point':
                 vis["payload"].set_transform(tf.translation_matrix(ppos))
            else:
                print('shape doesn\'t exist!')
                exit()
            for id in uavs.keys():
                quadsphere  = Quadspheres[id]
                constSphere = constSpheres[id]
                cable       = cables[id]
                muMaterial  = mushape[id]
                FdMaterial  = Fdshape[id]
                state       = states[id][tick]
                mu          = mus[id][tick]
                Fd          = Fds[id][tick]
                if len(uavs.keys()) > 1:  
                    hplanePerId = hplanes[id]
                    hpsPerId    = hps[id]
                    normal      = normals[id]

                vis["Quad"+id].set_object(quadsphere)
                vis["Quad"+id].set_transform(tf.translation_matrix(state[0:3]).dot(
                tf.quaternion_matrix(state[6:10])))
                
                vis["contrSph"+id].set_object(constSphere)
                vis["contrSph"+id].set_transform(tf.translation_matrix(state[0:3]).dot(
                tf.quaternion_matrix(state[6:10])))

                Fdpos  = np.linspace(ppos, ppos+2*Fd[0:3], num=2).T
                vis["Fd"+id].set_object(g.Line(g.PointsGeometry(Fdpos), material=FdMaterial))
                
                if ploadShape == 'triangle' or ploadShape == 'rod' or ploadShape == 'cuboid':
                    pRot = rn.to_matrix(plstate[6:10])
                    p0 = ppos + pRot@uavs[id]['att']
                elif ploadShape == 'point':
                    p0 = ppos
                else:
                    print('wrong shape!')
                    exit()
                cablePos = np.linspace(p0, state[0:3], num=2).T
                vis["cable"+id].set_object(g.Line(g.PointsGeometry(cablePos), material=cable))

                try: 
                    muvec = np.linspace(p0, p0 + muShape['scale']*(mu), num=2).T
                except:
                    print('norm mu is zero!')
                    raise 
                    break
                vis["mu"+id].set_object(g.Line(g.PointsGeometry(muvec), material=muMaterial))
                if len(uavs.keys()) > 1:
                    for hpsKey in hpsPerId.keys():
                        hp = hpsPerId[hpsKey][tick,:]
                        n = hp[0:3]
                        a = hp[-1]
                        if np.linalg.norm(n) > 0:
                            R = tf.identity_matrix()
                            z_fixed = [0,0,1]
                            z = normVec(n)
                            q = rn.vector_vector_rotation(z_fixed, z) 
                            R1 = rn.to_matrix(q)
                            R[:3, 0] = R1[:,0]
                            R[:3, 1] = R1[:,1]
                            R[:3, 2] = R1[:,2]
                            R[:3, 3] = p0
                            planeObj = hplanePerId[hpsKey][0]
                            planeMat = hplanePerId[hpsKey][1]

                            vis["p"+str(hpsKey)+id].set_object(planeObj, planeMat)
                            vis["p"+str(hpsKey)+id].set_transform(R)
                            
                            # draw normals
                            normalMat = normal[hpsKey]
                            n_ = np.linspace(p0, 0.2*n/np.linalg.norm(n)+p0, num=2).T
                            vis["n"+str(hpsKey)+id].set_object(g.Line(g.PointsGeometry(n_), material=normalMat))
                        else:
                            vis["p"+str(hpsKey)+id].delete()
                            vis["n"+str(hpsKey)+id].delete()
            tick+=1
            time.sleep(visProps["timestep"])
                  
