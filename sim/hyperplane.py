import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d 
from itertools import permutations, combinations, chain
np.set_printoptions(suppress=True)

def normVec(n):
    return n / np.linalg.norm(n)


# pos1 = np.array([0, -0.4275, 0.74045172])
# pos2 = np.array([0, 0.4275, 0.74045172])
# pairsinIds = list(permutations(ids, 2))
# pos1 = np.array([0, -0.29242722,  0.80343719])
pos1 = np.array([0, -0.5, 0.8660254])
pos2 = np.array([0.4330127, 0.25, 0.8660254])
pos3 = np.array([-0.4330127, 0.25, 0.8660254])
load = np.array([0,0,0])
poses = [pos1, pos2, pos3]
dx = np.array([0.05, 0, 0]) # distance x from center
dy = np.array([0, 0.05, 0]) # distance y from center
dz = np.array([0, 0, 0.05]) # distance z from center

cubeV1 =  np.array([pos1+dx+dy+dz, pos1-dx+dy+dz, pos1+dx-dy+dz, pos1-dx-dy+dz, pos1+dx+dy-dz, pos1-dx+dy-dz, pos1+dx-dy-dz, pos1-dx-dy-dz])
cubeV2 =  np.array([pos2+dx+dy+dz, pos2-dx+dy+dz, pos2+dx-dy+dz, pos2-dx-dy+dz, pos2+dx+dy-dz, pos2-dx+dy-dz, pos2+dx-dy-dz, pos2-dx-dy-dz])
cubeV3 =  np.array([pos3+dx+dy+dz, pos3-dx+dy+dz, pos3+dx-dy+dz, pos3-dx-dy+dz, pos3+dx+dy-dz, pos3-dx+dy-dz, pos3+dx-dy-dz, pos3-dx-dy-dz])
loadV4 =  np.array([load+dx+dy+dz, load-dx+dy+dz, load+dx-dy+dz, load-dx-dy+dz, load+dx+dy-dz, load-dx+dy-dz, load+dx-dy-dz, load-dx-dy-dz])
n_stack = np.empty((1,3))
a_s = []
pairs = list(permutations(poses,2))
for pair in pairs:
    pos_1 = pair[0]
    pos_2 = pair[1]
    dx = np.array([0.005, 0, 0]) # distance x from center
    dy = np.array([0, 0.005, 0]) # distance y from center
    dz = np.array([0, 0, 0.005]) # distance z from center
    cubeVer1 =  np.array([pos_1+dx+dy+dz, pos_1-dx+dy+dz, pos_1+dx-dy+dz, pos_1-dx-dy+dz, pos_1+dx+dy-dz, pos_1-dx+dy-dz, pos_1+dx-dy-dz, pos_1-dx-dy-dz])
    cubeVer2 =  np.array([pos_2+dx+dy+dz, pos_2-dx+dy+dz, pos_2+dx-dy+dz, pos_2-dx-dy+dz, pos_2+dx+dy-dz, pos_2-dx+dy-dz, pos_2+dx-dy-dz, pos_2-dx-dy-dz])
    # cubeload =  np.array([pload+dx+dy+dz, pload-dx+dy+dz, pload+dx-dy+dz, pload-dx-dy+dz, pload+dx+dy-dz, pload-dx+dy-dz, pload+dx-dy-dz, pload-dx-dy-dz])

    constraints = []
    n = cp.Variable(3,)
    a = cp.Variable()
    Q = np.eye(3)
    objective = cp.Minimize(cp.quad_form(n, Q))

    for i in range(len(cubeVer1)):
        constraints.append([n.T@cubeVer1[i,:] + a <= -1])
    for i in range(len(cubeVer2)):
        constraints.append([n.T@cubeVer2[i,:] + a >= 1])
    # for i in range(len(cubeload)):
    #     constraints.append([n.T@cubeload[i,:] + a >= 1])
    # for i in range(len(cubeload)):
    #     constraints.append([n.T@cubeload[i,:] + a >=  1])
    constraints = list(chain.from_iterable(constraints))
    prob = cp.Problem(objective, constraints)
    prob.solve(verbose=False)
    n =  normVec(n.value)
    a = np.array([a.value])[0]
    n_stack = np.vstack((n_stack, n.reshape(1,3)))
    for i in range(len(cubeVer1)):
        pdiff = cubeVer1[i] - pos_1
        a = max(n.T@pdiff, a)
    a_s.append(a)
    print('hyperplane normal vec: \n',n)
    print('a: ',a)

n_stack = np.delete(n_stack,  0, 0)

fig = plt.figure()
fig.tight_layout()
ax = fig.add_subplot(autoscale_on=True,projection="3d")
ax.view_init(azim=0,elev=0)
ax.plot3D(cubeV1[:,0], cubeV1[:,1], cubeV1[:,2], 'r*',lw=1.5)
ax.plot3D(cubeV2[:,0], cubeV2[:,1], cubeV2[:,2], 'b*',lw=1.5)
ax.plot3D(cubeV3[:,0], cubeV3[:,1], cubeV3[:,2], 'g*',lw=1.5)
ax.plot3D(loadV4[:,0], loadV4[:,1], loadV4[:,2], 'g*',lw=1.5)
ax.plot3D(load[0], load[1], load[2], 'g*',lw=1.5)

ax.quiver(0,0,0, pos1[0], pos1[1], pos1[2] ,color = 'r')
ax.quiver(0,0,0, pos2[0], pos2[1], pos2[2] ,color = 'b')
ax.quiver(0,0,0, pos3[0], pos3[1], pos3[2] ,color = 'g')
for i in range(len(n_stack)):
    n = normVec(n_stack[i,:]) 
    ax.quiver( 0,0,0, n[0], n[1], n[2] ,color = 'k')
    a = n_stack[i,0]
    b = n_stack[i,1]
    c = n_stack[i,2]
    d = a_s[i]
    x = np.linspace(-1, 1)
    y = np.linspace(-1, 1)
    x, y = np.meshgrid(x, y)
    z = -(a/c)*x - (b/c)*y - d
    print(z)
    z[z>2]=2
    z[z<-2]= 2
    # print(np.amax(z), np.amin(z))
    ax.plot_surface(x, y, z, rstride=1, cstride=1, alpha=0.2)
    # ax.plot_surface(X, Y, Z,rstride=1, cstride=1, alpha=0.2)

# ax.set_xlim3d([-0.1, 0.1])
# ax.set_ylim3d([-0.1, 0.1])
# ax.set_zlim3d([-0.1, 0.1])
plt.show()