import cfusdlog
import numpy as np
import rowan #uses q, x, y, z

# https://mariogc.com/post/angular-velocity-quaternions/
# changed to match the [x,y,z,w] format
def angular_velocities(q1, q2, dt):
    return (2 / dt) * np.array([
        q1[3]*q2[0] - q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1],
        q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3] - q1[2]*q2[0],
        q1[3]*q2[2] - q1[0]*q2[1] + q1[1]*q2[1] - q1[2]*q2[3]])

def main(args=None):

    logData = cfusdlog.decode("cf6_06")['fixedFrequency']

    quat = np.array([    logData['stateEstimate.pqw'],
                         logData['stateEstimate.pqx'],
                         logData['stateEstimate.pqy'],
                         logData['stateEstimate.pqz']]).T
    # quat = rowan.normalize(quat)

    omega = np.array([    logData['stateEstimate.pwx'],
                         logData['stateEstimate.pwy'],
                         logData['stateEstimate.pwz']]).T

    time = logData['timestamp'] / 1000
    dt = np.diff(time)
    dt = dt.reshape(dt.shape[0], 1)

    quat_int = rowan.calculus.integrate(quat[1:,:], omega[:-1,:], dt)

    print(rowan.allclose(quat[1:], quat_int))
    print(rowan.geometry.distance(quat[1:], quat_int))

    print(angular_velocities(quat[0], quat[1], dt[0]))
    print(omega[0])

    print(quat[0], quat[1], omega[0], dt[0])

    ## generate test data
    q1 = np.array([1,0,0,0])
    omega = np.array([10, 20, 30])
    dt = 0.01
    q2 = rowan.calculus.integrate(q1, omega, dt)
    print(q1, omega, dt, q2)

    # print(quat_int)

if __name__ == '__main__':
    main()