# Standard Library
import time
import argparse
import logging
from pathlib import Path
import os
import json
import math
# External Library
import cv2
import numpy as np
from fairino import Robot

PROJECT_ROOT = Path(__file__).resolve().parent.parent

def pose_estimation(image: np.ndarray,matrix_coefficients_path: (str), distortion_coefficients_path: (str),marker_length: float = 0.07, aruco_dict_type: int = cv2.aruco.DICT_5X5_100) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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
            [-marker_length/2,  marker_length/2, 0],
            [ marker_length/2,  marker_length/2, 0],
            [ marker_length/2, -marker_length/2, 0],
            [-marker_length/2, -marker_length/2, 0]
        ], dtype=np.float32)

        for i, corner in enumerate(corners):
            img_points = corner.reshape(4, 2).astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(obj_points, img_points,
                                               matrix_coefficients, distortion_coefficients)

            if success:
                cv2.drawFrameAxes(image, matrix_coefficients, distortion_coefficients,
                                  rvec, tvec, marker_length * 1.5)

                logging.info(f"Marker ID {ids[i][0]} - rvec: {rvec.flatten()}, tvec: {tvec.flatten()}")

    return image, rvec, tvec

def capture_image(output: str):
    """
    !Capture an image and saves it to the "saved_pictures" folder

    @output (str): Name of the out image     (example.jpg)
    """
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)


    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    time.sleep(5)
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")

    else: 
        cv2.imshow("Captured iamge", frame)

        cv2.imwrite(PROJECT_ROOT/ "saved_pictures" / output, frame)

    cap.release()
    cv2.destroyAllWindows


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

    #print("T_base_ee calculated:")
    #print(T_base_ee)

    #JSON_PATH = PROJECT_ROOT / "data" / "transforms.json"

    return T_base_ee

#capture_image("images1.jpg")
#calculate_T_base_ee(-549.362,42.653,561.389,6.518,54.629,159.229)


def transform_to_pose6dof_deg(T: np.ndarray):
    """
    Convert a 4x4 homogeneous transform to [x, y, z, rx, ry, rz],
    where translation is in meters and rotation is in degrees (roll, pitch, yaw).
    
    Input:
        T: 4x4 numpy array (homogeneous transform)
    
    Returns:
        pose_6dof: np.array([x, y, z, rx_deg, ry_deg, rz_deg])
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
    color = (0, 255, 0)  # green
    radius = 10
    thickness = -1  # filled circle

    # Define four points (corners of a workspace)
    points = [
        (100, 100),  # top-left
        (540, 100),  # top-right
        (100, 380),  # bottom-left
        (540, 380)   # bottom-right
    ]
    cap = cv2.VideoCapture(1)
    t_cam_marker = None
    t_base_ee = None
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    JSON_PATH = PROJECT_ROOT / "data" / "transforms.json"
    while True:
        ret, frame = cap.read()
        if not ret:
            break


        pose = []
        INPUT_DIR = PROJECT_ROOT / "input"
        #image_full_path = PROJECT_ROOT / "saved_pictures" / image_path
        #frame = cv2.imread(image_full_path)
        in_frame = frame.copy()
        matrix_coefficients_path = INPUT_DIR / "calibration_matrix.npy"
        distortion_coefficients_path = INPUT_DIR / "distortion_coefficients.npy"
        estimated_frame, rvec, tvec = pose_estimation(
                    image=in_frame,
                    matrix_coefficients_path=matrix_coefficients_path,
                    distortion_coefficients_path=distortion_coefficients_path
            )
        cv2.imshow("real time", estimated_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            t_cam_marker = calculate_T_camera_marker(frame)
            print("t_cam_marker: ", t_cam_marker)
            robot = Robot.RPC('192.168.58.2')
            error, pose = robot.GetActualTCPPose()
            t_base_ee = calculate_T_base_ee(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])
            robot.CloseRPC()
            print("t_base_ee: ", t_base_ee)
        elif key == ord('s'):
            if t_cam_marker is None or t_base_ee is None:
                print("⚠️  No data to save yet — press 'c' first!")
                continue

            # Read existing data if file exists
            if JSON_PATH.exists():
                with open(JSON_PATH, "r") as f:
                    data = json.load(f)
            else:
                data = []

            # Append new pair (convert np arrays to list)
            data.append({
                "T_cam_marker": np.array(t_cam_marker).tolist(),
                "T_base_ee": np.array(t_base_ee).tolist()
            })

            # Write back to file
            with open(JSON_PATH, "w") as f:
                json.dump(data, f, indent=4)
            
            print(f"✅ Saved pair to {JSON_PATH.name}")

        elif key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

#capture_image("circle1.png")

             
        



    