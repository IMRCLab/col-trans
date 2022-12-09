import numpy as np
import time
import rowan as rn
import yaml
# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
np.set_printoptions(suppress=True)


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
    with open('config.yaml') as f:
        meshcatdata = yaml.load(f, Loader=yaml.FullLoader)

    with open('configData.yaml') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    
    uavs         = data['robots']
    # print(uavs)
    payload      = data['payload']
    
    visProps     = meshcatdata['meshcat']
    robotshape   = meshcatdata['robot']['shape']
    colors       = meshcatdata['robot']['colors']
    hpShape      = meshcatdata['robot']['hps']
    muShape      = meshcatdata['robot']['mu']
    cableShape   = meshcatdata['robot']['cable']
    constrShape  = meshcatdata['robot']['constrSphere']
    ploadShape   = meshcatdata['payload']

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
    # shapes
    Quadspheres  = {}
    constSpheres = {}
    hplanes      = {}
    normals      = {}
    cables       = {}
    mushape      = {}
    quadNums     = len(uavs.keys())
    for id, uav, qNum in zip(uavs.keys(), uavs.values(), range(quadNums)):
        # File paths for states
        stateFilePath = uav['state']
        hpsFilePaths  = uav['hps']
        musFilePath   = uav['mu']

        
        # Add states for uav
        states[id]    = np.loadtxt(stateFilePath, delimiter=',')
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
            hptmp[i]       = np.loadtxt(hpFilePath, delimiter=',')
            hps[id]        = hptmp.copy()
            # Add materials for hyperplanes and normals
            hpcolor        = colors[qNum]
            hpopacity      = hpShape['opacity']
            size           = hpShape['size']
            linewidthNorm  = hpShape['linewidth']
            
            normaltmp[i]   = g.LineBasicMaterial(linewidth=linewidthNorm, color=hpcolor)
            hplanetmp[i]   =  (Plane(width=size[0], height=size[1]), 
                              g.MeshBasicMaterial(opacity=hpopacity, color=hpcolor))
            hplanes[id]    = hplanetmp 
            normals[id]    = normaltmp
        ##
        # Add state for mu vector
        mus[id]            = np.loadtxt(musFilePath, delimiter=',')
        # Add shape for mu vector 
        mushape[id]        =  g.LineBasicMaterial(linewidth=muShape['linewidth'], color=colors[qNum])
        # Add shape for cable 
        cables[id]         = g.LineBasicMaterial(linewidth=cableShape['linewidth'], color=cableShape['color'])
    
    plstatePath = payload
    plstates     = np.loadtxt(plstatePath, delimiter=',')
    plproperties = meshcatdata['payload']
    vis["payload"].set_object(g.Mesh(g.Sphere(plproperties['radius']), g.MeshLambertMaterial(color=plproperties['color'])))
   
    while True:    
        tick = 0
        for plstate in plstates:
            # print(plstates.shape)
            ppos = plstate[0:3]
            vis["payload"].set_transform(
                           tf.translation_matrix(ppos))

            for id in uavs.keys():
                quadsphere  = Quadspheres[id]
                constSphere = constSpheres[id]
                cable       = cables[id]
                muMaterial  = mushape[id]
                hplanePerId = hplanes[id]
                hpsPerId    = hps[id]
                normal      = normals[id]
                state       = states[id][tick,:]
                mu          = mus[id][tick,:]

                vis["Quad"+id].set_object(quadsphere)
                vis["Quad"+id].set_transform(tf.translation_matrix(state[0:3]).dot(
                tf.quaternion_matrix(state[6:10])))
                
                vis["contrSph"+id].set_object(constSphere)
                vis["contrSph"+id].set_transform(tf.translation_matrix(state[0:3]).dot(
                tf.quaternion_matrix(state[6:10])))

                cablePos = np.linspace(ppos, state[0:3], num=10).T
                vis["cable"+id].set_object(g.Line(g.PointsGeometry(cablePos), material=cable))

                # normalize mu because they are very small in values
                try: 
                    muvec = np.linspace(ppos, ppos + muShape['scale']*(mu), num=2).T
                except:
                    print('norm mu is zero!')
                    raise 
                    break
                vis["mu"+id].set_object(g.Line(g.PointsGeometry(muvec), material=muMaterial))
                for hpsKey in hpsPerId.keys():
                    hp = hpsPerId[hpsKey][tick,:]
                    n = hp[0:3]
                    a = hp[-1]
                    R = tf.identity_matrix()
                    z_fixed = [0,0,1]
                    z = normVec(n)
                    q = rn.vector_vector_rotation(z_fixed, z) 
                    R1 = rn.to_matrix(q)
                    R[:3, 0] = R1[:,0]
                    R[:3, 1] = R1[:,1]
                    R[:3, 2] = R1[:,2]
                    R[:3, 3] = ppos
                    planeObj = hplanePerId[hpsKey][0]
                    planeMat = hplanePerId[hpsKey][1]

                    vis["p"+str(hpsKey)+id].set_object(planeObj, planeMat)
                    vis["p"+str(hpsKey)+id].set_transform(R)
                    
                    # draw normals
                    normalMat = normal[hpsKey]
                    n_ = np.linspace(ppos, n+ppos, num=10).T
                    vis["n"+str(hpsKey)+id].set_object(g.Line(g.PointsGeometry(n_), material=normalMat))
            tick+=1
            time.sleep(visProps["timestep"])
                  