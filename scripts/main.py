#Standard library
import json
import numpy as np
from pathlib import Path
import math

#external library
import cv2
from scipy.spatial.transform import Rotation

def calibrate_eye_hand(T_base_ee_list: list[np.ndarray], T_cam_marker_list: list[np.ndarray], cal_val: int, eye_to_hand: bool=True) -> np.ndarray:
    """
    !Function to calculate 4x4 cam to base transform matrix and test matrix accuracy

    @T_base_ee_list (list[np.ndarray]): list of T_base_ee matrices
    @T_cam_marker_list (list[np.ndarray]): list of T_cam_marker matrices
    @cal_val (int): Decide numbers of matrixes input pair to calibrate on(choose a number that leave the at least 2 matrix pairs of the test), the rest will go to test
    @eye_to_hand (bool): Whether calibration is eye-on-hand or eye-to-hand(Default = True)
    """

    R_ee2base = [T[:3, :3] for T in T_base_ee_list]
    t_ee2base = [T[:3, 3].reshape(3, 1) for T in T_base_ee_list]

    R_marker2cam = [T[:3, :3] for T in T_cam_marker_list]
    t_marker2cam = [T[:3, 3].reshape(3, 1) for T in T_cam_marker_list]

    if eye_to_hand:
        # Convert from ee→base to base→ee coordinates
        R_base2ee, t_base2ee, T_ee_base_list = [], [], []

        for R, t in zip(R_ee2base, t_ee2base):
            R_b2g = R.T                     #Reshape t_ee2base to (3,1) NOT (3,)
            t_b2g = -R_b2g @ t

            # Save individual parts
            R_base2ee.append(R_b2g)
            t_base2ee.append(t_b2g)

            # Build homogeneous transform
            T_ee_base = np.eye(4)
            T_ee_base[:3, :3] = R_b2g
            T_ee_base[:3, 3] = t_b2g.flatten()  # flatten (3,1) → (3,)
            T_ee_base_list.append(T_ee_base)
            
        # calibrate
        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            R_gripper2base=R_base2ee[0:cal_val],
            t_gripper2base=t_base2ee[0:cal_val],
            R_target2cam=R_marker2cam[0:cal_val],
            t_target2cam=t_marker2cam[0:cal_val],
            method = cv2.CALIB_HAND_EYE_TSAI
        )

        T_base_cam = np.eye(4)
        T_base_cam[:3,:3] = R_cam2base
        T_base_cam[:3, 3] = t_cam2base.flatten()
        #T_base_cam[2, 3] = 0.55

        
    # Test matrix accuracy
    trans_errors = []
    rot_errors = []
    z_errors = []
    x_errors = []
    y_errors = []
    for i in range(len(T_base_ee_list)-1):
        lhs = T_ee_base_list[i] @ T_base_cam @ T_cam_marker_list[i]
        rhs = T_ee_base_list[i+1] @ T_base_cam @ T_cam_marker_list[i+1]
        
        # Translation error (Euclidean distance)
        t_err = np.linalg.norm(lhs[:3, 3] - rhs[:3, 3])
        #print("t_err:", t_err)
        trans_errors.append(t_err)
        

        z_error = np.linalg.norm(lhs[2,3]-rhs[2,3])
        z_errors.append(z_error)
        y_error = np.linalg.norm(lhs[1,3]-rhs[1,3])
        y_errors.append(y_error)
        x_error = np.linalg.norm(lhs[0,3]-rhs[0,3])
        x_errors.append(x_error)
        # Rotation error (angle difference in degrees)
        R_err = Rotation.from_matrix(lhs[:3, :3].T @ rhs[:3, :3])
        rot_angle = R_err.magnitude() * 180/np.pi
        #print("rot_angle", rot_angle)
        rot_errors.append(rot_angle)
    print("Mean translation error (m):", np.mean(trans_errors))
    print("Mean rotation error (deg):", np.mean(rot_errors))
    print("z translation error [m]: ", np.mean(z_errors))
    print("y translation error [m]: ", np.mean(y_errors))
    print("x translation error [m]: ", np.mean(x_errors))
   
    return T_base_cam

def detect_circle(input_path: str) -> list[np.ndarray]:
    """
    !Detect circle from input image path
    @input_path (str): input image path
    """
    img = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
    print("image_size: ", img.shape)
    assert img is not None, "file could not be read, check with os.path.exists()"
    #img = cv2.medianBlur(img,5)
    img = cv2.GaussianBlur(img, (7, 7), 0)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
                                param1=400,param2=35,minRadius=0,maxRadius=0)
    
    circles = np.uint16(np.around(circles))
    circles = sorted(circles[0, :], key=lambda c: c[0])

    print("circles shape: \n", circles[0].shape)
    for idx, i in enumerate(circles, start=1):  # start=1 labels from 1
        x, y, r = int(i[0]), int(i[1]), int(i[2])
        
        # Draw the outer circle
        cv2.circle(cimg, (x, y), r, (0, 255, 0), 2)
        
        # Draw the center
        cv2.circle(cimg, (x, y), 2, (0, 0, 255), 3)
        
        # Put label
        cv2.putText(
            cimg,                  # image
            str(idx),              # text
            (x + r + 5, y),        # position (slightly to the right of the circle)
            cv2.FONT_HERSHEY_SIMPLEX,  # font
            0.7,                   # font scale
            (255, 0, 0),           # color (B,G,R)
            2                      # thickness
        )

    cv2.namedWindow('detected circles', cv2.WINDOW_NORMAL)
    cv2.imshow('detected circles',cimg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return circles

def circles_poses_solvePnP(circles, radius_real, mtx, dist) -> list[np.ndarray]:
    """
    ! Compute the circle pose in the camera frame
    @circles (list[np.ndarray]): list of detected circle (x,y,r)
    """
    circles = np.asarray(circles, dtype=np.float64)
    poses = []
    
    # 4 points on the circle in circle local frame (plane z=0)
    # top, bottom, left, right (X,Y,Z)
    obj_pts_local = np.array([
        [-radius_real, 0, 0],  # left
        [ radius_real, 0, 0],  # right
        [0, -radius_real, 0],  # top
        [0,  radius_real, 0]   # bottom
    ], dtype=np.float64)
    
    for x, y, r in circles:
        # Map 4 points to image pixels
        # Use detected pixel center and radius as scale
        img_pts = np.array([
            [x - r, y],  # left
            [x + r, y],  # right
            [x, y - r],  # top
            [x, y + r]   # bottom
        ], dtype=np.float64)
        
        # SolvePnP to get rotation + translation
        success, rvec, tvec = cv2.solvePnP(obj_pts_local, img_pts, mtx, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            raise RuntimeError(f"solvePnP failed for circle at pixel ({x},{y})")
        
        # Convert rvec to rotation matrix
        T_cam_circle = np.eye(4)
        R, _ = cv2.Rodrigues(rvec)
        T_cam_circle[:3,:3] = R
        T_cam_circle[:3, 3] = tvec.flatten()
        poses.append(T_cam_circle)
        
    return poses

def transform_to_pose6dof_deg(T: np.ndarray) -> np.ndarray:
    """
    !Convert a 4x4 homogeneous transform to [x, y, z, rx, ry, rz]
    @T (np.ndarray): input 4x4 transform matrix
    """
    if T.shape != (4, 4):
        raise ValueError("Input must be a 4x4 homogeneous matrix")

    R = T[:3, :3]
    t = T[:3, 3]

    # Ensure R is a proper rotation (orthonormalize if necessary)
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    # Extract Euler angles (roll-pitch-yaw, X-Y-Z) in radians
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0

    # Convert radians → degrees
    rx, ry, rz = map(math.degrees, [roll, pitch, yaw])

    # Output [x, y, z, rx, ry, rz]
    pose_6dof = np.array([t[0], t[1], t[2], rx, ry, rz])
    return pose_6dof

def main():
    '''
    !Pipeline: From taking input image to detect the circle pose in robot base frame
    '''
    ### Load the image and camera matrix
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    input_path = r"C:\Users\Admin_PC\Desktop\robot\HEC_modbus\saved_pictures\calib_000.png"
    mtx = np.load(PROJECT_ROOT / "input" / "calibration_matrix.npy")
    dist = np.load(PROJECT_ROOT / "input" / "distortion_coefficients.npy")

    ### Load the data for hand eye calibration
    JSON_PATH = PROJECT_ROOT / "data" / "transforms.json"
    with open(JSON_PATH, "r") as f:
        data = json.load(f)
    T_base_ee_list = [np.array(item["T_base_ee"]) for item in data]
    T_cam_marker_list = [np.array(item["T_cam_marker"]) for item in data]
    #print("T_base_ee_list_shape: ", T_base_ee_list[0].shape)
    cal_val = len(T_base_ee_list)
    # Get the desired matrix

    ### Hand to eye calibration
    T_base_cam = calibrate_eye_hand(T_base_ee_list, T_cam_marker_list,cal_val)
    print("Samples used: \n", cal_val)
    print("T_base_cam: \n", T_base_cam)
    # circle shape = (1,11,3), access by circles[0][i][0] = x, circles[0][i][1] = y, circles[0][i][2] = r


    ### Detect circle pose in robot frame (position)

    circles = detect_circle(input_path)
    T_cam_circle_list = circles_poses_solvePnP(circles, 0.014, mtx, dist)
    # Get 3 circle positions 
    for i in range(5,6):
        print("poses: \n", T_cam_circle_list[i])
        T_base_circle = T_base_cam @ T_cam_circle_list[i]
        position = transform_to_pose6dof_deg(T_base_circle)
        # Rescale from m to mm
        position[0] *=1000
        position[1] *=1000
        position[2] *=1000
        print("position: ", position)

if __name__ == "__main__":
    main()