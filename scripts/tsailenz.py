import numpy as np
from scipy.spatial.transform import Rotation as R

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def tsai_lenz(A, B):
    n = len(A)
    M = np.zeros((3*n, 3))
    b = np.zeros(3*n)
    for i in range(n):
        Ra, Rb = A[i][:3, :3], B[i][:3, :3]
        alpha = R.from_matrix(Ra).as_rotvec()
        beta = R.from_matrix(Rb).as_rotvec()
        M[3*i:3*i+3, :] = skew(alpha + beta)
        b[3*i:3*i+3] = beta - alpha
    rx, *_ = np.linalg.lstsq(M, b, rcond=None)
    R_x = R.from_rotvec(rx).as_matrix()

    C, d = [], []
    for i in range(n):
        Ra, ta = A[i][:3, :3], A[i][:3, 3]
        Rb, tb = B[i][:3, :3], B[i][:3, 3]
        C.append(Ra - np.eye(3))
        d.append(R_x @ tb - ta)
    C, d = np.vstack(C), np.hstack(d)
    t_x, *_ = np.linalg.lstsq(C, d, rcond=None)

    X = np.eye(4)
    X[:3, :3] = R_x
    X[:3, 3] = t_x
    return X

# ---- Test for eye-to-base ----
def random_transform():
    T = np.eye(4)
    T[:3, :3] = R.random().as_matrix()
    T[:3, 3] = np.random.randn(3)
    return T


def random_transform_limited():
    # small random rotation (<= 15 degrees)
    axis = np.random.randn(3)
    axis /= np.linalg.norm(axis)
    theta = np.random.uniform(-np.pi/12, np.pi/12)
    Rm = R.from_rotvec(axis * theta).as_matrix()

    # small random translation (within 10 cm cube)
    t = np.random.uniform(-0.1, 0.1, 3)
    
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = t
    return T

# True transforms
X_true = random_transform()  # Base->Camera
Y_true = random_transform()  # EndEffector->Marker

# Generate synthetic dataset
poses_robot, poses_cam = [], []
for _ in range(20):
    T_base_ee = random_transform_limited()
    T_cam_marker = np.linalg.inv(X_true) @ T_base_ee @ Y_true
    poses_robot.append(T_base_ee)
    poses_cam.append(T_cam_marker)


# Build relative motions
A, B = [], []
for i in range(19):
    A_i = np.linalg.inv(poses_robot[i]) @ poses_robot[i+1]
    B_i = poses_cam[i] @ np.linalg.inv(poses_cam[i+1])
    A.append(A_i)
    B.append(B_i)

X_est = tsai_lenz(A, B)
print("True Base->Camera:\n", X_true)
print("Estimated Base->Camera:\n", X_est)
print("Error:", np.linalg.norm(X_true - X_est))

print("True translation:", X_true[:3, 3])
print("Estimated translation:", X_est[:3, 3])

R_diff = X_true[:3, :3].T @ X_est[:3, :3]
angle_err = np.degrees(np.arccos((np.trace(R_diff) - 1) / 2))
print("Rotation error (deg):", angle_err)
