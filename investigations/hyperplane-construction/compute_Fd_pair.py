import numpy as np
import rowan
import cvxpy as cp

def skew(w):
    w = np.asarray(w).reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))

def main():
    Fd = np.array([0.000000, 0.000000, 0.165789])
    Md = np.array([0.001710, -0.000000, -0.000000])
    # Md = np.array([0.0, -0.000000, -0.000000])
    q = np.array([1.0, 0.000000, 0.000000, 0.000000])
    p1a = np.array([0.000000, 0.035500, 0.000000])
    # p2a = np.array([ -0.041000, -0.035500, 0.000000])
    p2a = np.array([0.000000, 0.035500, 0.000000])

    # rigid body case
    P = np.zeros((6, 6))
    P[0:3,0:3] = np.eye(3)
    P[0:3,3:6] = np.eye(3)
    P[3:6,0:3] = skew(p1a)
    P[3:6,3:6] = skew(p2a)
    print(P)
    # print(np.linalg.matrix_rank(P))
    P_inv = np.linalg.pinv(P)
    # P_inv = P.T @ np.linalg.inv(P@P.T)
    # print(P)

    print(P_inv)


    Rp = rowan.to_matrix(q)
    R_blockdiag = np.zeros((6, 6))
    R_blockdiag[0:3,0:3] = Rp
    R_blockdiag[3:6,3:6] = Rp

    forces_at_attachment_points = R_blockdiag @ P_inv @ np.concatenate((Rp.T @ Fd, Md))
    Fd1 = forces_at_attachment_points[0:3]
    Fd2 = forces_at_attachment_points[0:3]

    print("pinv")
    print(Fd1)
    print(Fd2)
    print(np.linalg.norm(Fd2+Fd1 - Fd))
    print(np.linalg.norm(skew(p1a) @ Rp.T @ Fd1 + skew(p2a) @ Rp.T @ Fd2 - Md))

    # optimization-based

    # see (22) in Lee's paper
    Fd1 = cp.Variable(3)
    Fd2 = cp.Variable(3)
    prob = cp.Problem(cp.Minimize(
        cp.norm(skew(p1a) @ Rp.T @ Fd1 + skew(p2a) @ Rp.T @ Fd2 - Md)),
            [
                Fd1 + Fd2 == Fd,
                # skew(p1a) @ Rp.T @ Fd1 + skew(p2a) @ Rp.T @ Fd2 == Md,
            ])
    prob.solve(verbose=False)
    Fd1 = Fd1.value
    Fd2 = Fd2.value

    print("opt")
    print(Fd1)
    print(Fd2)
    print(np.linalg.norm(Fd2+Fd1 - Fd))
    print(np.linalg.norm(skew(p1a) @ Rp.T @ Fd1 + skew(p2a) @ Rp.T @ Fd2 - Md))


if __name__ == '__main__':
    main()


