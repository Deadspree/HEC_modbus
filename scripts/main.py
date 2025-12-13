#Standard library
import json
import numpy as np
from pathlib import Path
import math
import time
#external library
import cv2
from scipy.spatial.transform import Rotation
from transform_matrix_modbus import calculate_T_cam_charuco
from pupil_apriltags import Detector
from fairino import Robot
PROJECT_ROOT = Path(__file__).resolve().parent.parent

def pose_estimation(image: np.ndarray,matrix_coefficients_path: (str), distortion_coefficients_path: (str),marker_length: float = 0.10, aruco_dict_type: int = cv2.aruco.DICT_5X5_100) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    !Perform pose estimation on each frame

    @image (np.ndarry): Input image(BGR format)
    @matrix_coefficients_path (str): Path to load the matrix_coefficients from camera calibration
    @distortion_coefficients_path (str): Path to load the distortion_coefficients from camera calibration
    @marker_length (float): Length of marker
    @aruco_dict_type (int): Aruco dictionrary type used for detection
    """
    
    matrix_coefficients = np.load(matrix_coefficients_path)
    distortion_coefficients = np.load(distortion_coefficients_path)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Detect markers
    corners, ids, rejected = detector.detectMarkers(gray)
    rvec, tvec = None, None
    if ids is not None and len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        # 3D object points of a square marker (center at origin, lying on z=0 plane)
        
        obj_points = np.array([
            [0, marker_length, 0],    # TL -> Top-left in image
            [marker_length, marker_length, 0],  # TR
            [marker_length, 0, 0],    # BR
            [0, 0, 0]                 # BL -> origin
        ], dtype=np.float32)
        """
        obj_points = np.array([
            [-marker_length/2, marker_length / 2, 0], #L TL
            [marker_length/2, marker_length/2, 0],  # TR
            [marker_length/2, -marker_length/2, 0],    # BR
            [-marker_length/2, -marker_length/2, 0]                 # BL -> origin
        ], dtype=np.float32)
        """
        for i, corner in enumerate(corners):
            img_points = corner.reshape(4, 2).astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(obj_points, img_points,
                                               matrix_coefficients, distortion_coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE)

            if success:
                cv2.drawFrameAxes(image, matrix_coefficients, distortion_coefficients,
                                  rvec, tvec, marker_length * 1.5)

                #logging.info(f"Marker ID {ids[i][0]} - rvec: {rvec.flatten()}, tvec: {tvec.flatten()}")

    return image, rvec, tvec


def calculate_T_camera_marker(image_path) -> np.ndarray:
    """
    !Calculate the Tcamera_marker by using the captured image from "save_pictures" folder and save the matrix to transforms.json

    @image_path (str): input image name in the saved pictures folder  (example.jpg)
    """
    T_cam_marker = np.eye(4)
    INPUT_DIR = PROJECT_ROOT / "input"
    #image_full_path = PROJECT_ROOT / "saved_pictures" / image_path
    #frame = cv2.imread(image_full_path)
    frame = image_path
    matrix_coefficients_path = INPUT_DIR / "calibration_matrix.npy"
    distortion_coefficients_path = INPUT_DIR / "distortion_coefficients.npy"
    estimated_frame, rvec, tvec = pose_estimation(
                image=frame,
                matrix_coefficients_path=matrix_coefficients_path,
                distortion_coefficients_path=distortion_coefficients_path
            )
    print("rvec: ", rvec)
    print("tvec: ", tvec)
    R_cm = None
    tvec1 = None
    rvec1 = rvec.flatten()
    tvec1 = tvec.flatten()
    R_cm, _ = cv2.Rodrigues(rvec1)
    T_cam_marker[:3, :3] = R_cm
    T_cam_marker[:3, 3] = tvec1
    #print("T_cam_marker calculated:")
    #print(T_cam_marker)

    return T_cam_marker

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
    img = cv2.GaussianBlur(img, (7,7), 0)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
                                param1=400,param2=35,minRadius = 0,maxRadius= 0)
    
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
    pose_6dof = np.array([t[0]*1000, t[1]*1000, t[2]*1000, rx, ry, rz])
    return pose_6dof

def compute_safe_top_down_pose(T_base_circle: np.ndarray, desired_tool_z=(0,0,-1.0)) -> np.ndarray:
    """
    Forces the pose so that the tool Z axis points in desired_tool_z direction
    (default: downward in base frame), keeps the same position, and keeps yaw
    by projecting the original X axis into the horizontal plane.
    """

    T = np.array(T_base_circle, dtype=float)
    R = T[:3, :3]
    pos = T[:3, 3]

    # normalize desired Z
    z_new = np.array(desired_tool_z, dtype=float)
    z_new = z_new / np.linalg.norm(z_new)

    # Project original X axis onto plane orthogonal to new Z
    x_old = R[:, 0]
    x_proj = x_old - np.dot(x_old, z_new) * z_new

    # Handle degenerate case (x_proj too small)
    if np.linalg.norm(x_proj) < 1e-6:
        # choose arbitrary X direction not parallel to Z
        if abs(z_new[2]) < 0.9:
            x_proj = np.array([0, 0, 1], dtype=float)
        else:
            x_proj = np.array([1, 0, 0], dtype=float)
        x_proj = x_proj - np.dot(x_proj, z_new) * z_new

    x_new = x_proj / np.linalg.norm(x_proj)
    y_new = np.cross(z_new, x_new)
    y_new = y_new / np.linalg.norm(y_new)

    # rebuild rotation
    R_new = np.column_stack((x_new, y_new, z_new))

    # build T_safe
    T_safe = np.eye(4)
    T_safe[:3, :3] = R_new
    T_safe[:3, 3] = pos

    return T_safe

def calculate_T_cam_circle(image: np.ndarray, x_to_corner: float, y_to_corner: float) ->np.ndarray:
    """
    !Calculate the pose of circle in the camera frame
    @image (np.ndarray): input frame
    @x_to_corner (float): object x coordinate in the marker frame(metres)
    @y_to_corner (float): object y coordinate in the camera frame(metres)
    """
    T_cam_marker = calculate_T_camera_marker(image)
    if T_cam_marker is None:
        return []
    R_marker2cam = T_cam_marker[:3,:3]
    P_circle_marker = np.array([x_to_corner, y_to_corner, 0.0, 1.0])
    P_circle_cam = T_cam_marker @ P_circle_marker
    circle_cam_x = P_circle_cam[0]
    circle_cam_y = P_circle_cam[1]
    circle_cam_z = P_circle_cam[2]
    R_circle2cam = R_marker2cam
    T_cam_circle = np.eye(4)
    T_cam_circle[:3,:3] = R_circle2cam
    T_cam_circle[:3,3] = [circle_cam_x,circle_cam_y,circle_cam_z]
    return T_cam_circle

def extract_mtx(mtx_full_path):
    mtx = np.load(mtx_full_path)
    fx = mtx[0,0]
    fy = mtx[1,1]
    cx = mtx[0,2]
    cy = mtx[1,2]
    return fx, fy, cx, cy

def april_tag_pose_estimation(img: np.ndarray,tag_size: int, tag_family: str = "tag36h11", mtx_path: str = "calibration_matrix.npy", dst_path: str = "distortion_coefficients.npy"):
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    mtx_full_path = PROJECT_ROOT / "input" / mtx_path
    dst_full_path = PROJECT_ROOT / "input" / dst_path
    res = img.copy()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at_detector = Detector(families=tag_family,
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
    fx, fy, cx, cy = extract_mtx(mtx_full_path)

    dst = np.load(dst_full_path)
    tags = at_detector.detect(
        img,
        estimate_tag_pose=True,
        camera_params=[fx, fy, cx, cy],
        tag_size=tag_size
    )
    rvecs, tvec = None, None
    # --- Draw XYZ axes ---
    for tag in tags:
        # Pose returned by the detector
        rvec = tag.pose_R
        tvec = tag.pose_t

        # Convert rotation matrix to axis-angle
        rvecs, _ = cv2.Rodrigues(rvec)

        # Axis length (visualization)
        axis_len = tag_size

        # Define axis endpoints in 3D
        axis_points = np.float32([
            [0, 0, 0],                # origin
            [axis_len, 0, 0],         # X axis (red)
            [0, axis_len, 0],         # Y axis (green)
            [0, 0, axis_len]          # Z axis (blue)
        ])

        imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec.squeeze(), 
                              np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]),
                              distCoeffs=dst)

        # FIX: convert to integer pixel coords
        imgpts = np.int32(imgpts.reshape(-1, 2))

        # Draw the axes
        cv2.line(res, tuple(imgpts[0]), tuple(imgpts[1]), (0,0,255), 3)  # X - red
        cv2.line(res, tuple(imgpts[0]), tuple(imgpts[2]), (0,255,0), 3)  # Y - green
        cv2.line(res, tuple(imgpts[0]), tuple(imgpts[3]), (255,0,0), 3)  # Z - blue
    return res, rvecs, tvec

def calculate_T_cam_april(image: np.ndarray):
    T_cam_april = np.eye(4)
    _, rvecs, tvec = april_tag_pose_estimation(image, tag_size=0.035)
    T_cam_april[:3,:3] = rvecs
    T_cam_april[:3,3] = tvec.flatten()
    return T_cam_april

def T_base_marker(T_base_cam: np.ndarray, T_cam_marker: np.ndarray, offset_x: float, offset_y: float, offset_z: float):
    T_base_marker = T_base_cam @ T_cam_marker
    T_base_marker[0,3] += offset_x   #0.003533
    T_base_marker[1, 3] += offset_y    #-0.004396
    T_base_marker[2,3] += offset_z     #0.0035
    T_base_marker = compute_safe_top_down_pose(T_base_marker)
    position = transform_to_pose6dof_deg(T_base_marker)
    return T_base_marker, position
def calculate_T_base_ee(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> np.ndarray:
    """
    !Calculate T_base_ee from the robot pose and save it to transforms.json

    @x (float): x coordinates in mm
    @y (float): y coordinates in mm
    @z (float): z coordinates in mm
    @rx (float): Euler angles degree compared to x axis
    @ry (float): Euler angles degree compared to y axis
    @rz (float): Euler angles degree compared to z axis
    """
    rx = math.radians(rx)
    ry = math.radians(ry)
    rz = math.radians(rz)

    # Rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(rx), -math.sin(rx)],
                    [0, math.sin(rx), math.cos(rx)]])
    
    R_y = np.array([[math.cos(ry), 0, math.sin(ry)],
                    [0, 1, 0],
                    [-math.sin(ry), 0, math.cos(ry)]])
    
    R_z = np.array([[math.cos(rz), -math.sin(rz), 0],
                    [math.sin(rz), math.cos(rz), 0],
                    [0, 0, 1]])

    # Combined rotation (Z * Y * X)
    R_ee = R_z @ R_y @ R_x
    scale = 0.001 #for metres conversion
    # Homogeneous transformation
    T_base_ee = np.eye(4)
    T_base_ee[:3, :3] = R_ee
    T_base_ee[:3, 3] = [x *scale , y *scale, z * scale]
    return T_base_ee


def main():
    '''
    !Pipeline: From taking input image to detect the circle pose in robot base frame
    '''
    input = r"C:\Users\Admin_PC\Desktop\robot\HEC_modbus\input\calib_000.jpg"
    image = cv2.imread(input)
    T_base_gripper = None
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.05)   # important small delay!
    error, pose = robot.GetActualTCPPose()
    T_base_gripper = calculate_T_base_ee(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])
    T_cam_marker = calculate_T_cam_april(image)
    print("T_cam_marker: ", T_cam_marker)
    #T_cam_circle = calculate_T_cam_circle(image, x_to_corner = -0.042, y_to_corner = -0.014)
    #print("T_cam_circle: \n", T_cam_circle)
    JSON_PATH = PROJECT_ROOT / "data" / "calibrated_matrix.json"
    with open(JSON_PATH, "r") as f:
        data = json.load(f)
    T_gripper_cam = np.array(data["T_gripper_cam"])
    #T_base_circle = T_base_cam @ T_cam_circle
    #print("T_base_circle: \n", T_base_circle)
    T_base_marker = T_base_gripper @  T_gripper_cam @ T_cam_marker
    #T_base_marker[0,3] += 0.0246
    #T_base_marker[1, 3] += -0.017472
    #T_base_marker[2,3] -= 0.093923
    print("T_base_marker: \n", T_base_marker)
    T_safe = compute_safe_top_down_pose(T_base_marker)
    print("T_safe =\n", T_safe)
    position_pick = transform_to_pose6dof_deg(T_safe)
    position_place = position_pick.copy()
    position_place[2] += 100 # increase z to pplace
    #position[2] += 130 # off set z to avoid collision
    #position[0] += 3.
    #position[1] -= 4
    print("position pick: \n", ",".join(map(str, position_pick.flatten())))
    print("position place: \n", ",".join(map(str, position_place.flatten())))
     # Calculate circle position in base frame for verification
    P_circle_marker = np.array([0.08, -0.058, 0.0, 1.0])
    P_base_circle = T_base_marker @ P_circle_marker

    print("circle position in base frame:\n", 
      ",".join(map(str, P_base_circle.flatten())))
    robot.CloseRPC()
if __name__ == "__main__":
    main()