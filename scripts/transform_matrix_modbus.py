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


    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    time.sleep(2)
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")

    else: 
        cv2.imshow("Captured iamge", frame)

        cv2.imwrite(PROJECT_ROOT/ "saved_pictures" / output, frame)

    cap.release()
    cv2.destroyAllWindows


def calculate_T_camera_marker(image_path: str):
    """
    !Calculate the Tcamera_marker by using the captured image from "save_pictures" folder and save the matrix to transforms.json

    @image_path (str): input image name in the saved pictures folder  (example.jpg)
    """
    T_cam_marker = np.eye(4)
    INPUT_DIR = PROJECT_ROOT / "input"
    image_full_path = PROJECT_ROOT / "saved_pictures" / image_path
    
    frame = cv2.imread(image_full_path)

    matrix_coefficients_path = INPUT_DIR / "calibration_matrix.npy"
    distortion_coefficients_path = INPUT_DIR / "distortion_coefficients.npy"
    estimated_frame, rvec, tvec = pose_estimation(
                image=frame,
                matrix_coefficients_path=matrix_coefficients_path,
                distortion_coefficients_path=distortion_coefficients_path,
            )
    print("rvec: ", rvec)
    print("tvec: ", tvec)
    rvec1 = rvec.flatten()
    tvec1 = tvec.flatten()
    R_cm, _ = cv2.Rodrigues(rvec1)
    T_cam_marker[:3, :3] = R_cm
    T_cam_marker[:3, 3] = tvec1
    print("T_cam_marker calculated:")
    print(T_cam_marker)

    JSON_PATH = PROJECT_ROOT / "data" / "transforms.json"
    # Load existing data if file exists
    # Load existing data or start empty
    if os.path.exists(JSON_PATH):
        with open(JSON_PATH, "r") as f:
            data = json.load(f)
    else:
        data = {}

    # Ensure "cam_marker" key exists
    if "cam_marker" not in data:
        data["cam_marker"] = []

    # Append the new pose
    data["cam_marker"].append(T_cam_marker.tolist())

    # Save back to JSON
    with open(JSON_PATH, "w") as f:
        json.dump(data, f, indent=4)

    print(f"Cam marker saved! Total transforms: {len(data['cam_marker'])}")


def calculate_T_base_ee(x: float, y: float, z: float, rx: float, ry: float, rz: float):
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

    # Homogeneous transformation
    T_base_ee = np.eye(4)
    T_base_ee[:3, :3] = R_ee
    T_base_ee[:3, 3] = [x, y, z]

    print("T_base_ee calculated:")
    print(T_base_ee)

    JSON_PATH = PROJECT_ROOT / "data" / "transforms.json"

    if os.path.exists(JSON_PATH):
        with open(JSON_PATH, "r") as f:
            data = json.load(f)
    else:
        data = {}

    # Ensure "base_ee" key exists
    if "base_ee" not in data:
        data["base_ee"] = []

    # Append the new pose
    data["base_ee"].append(T_base_ee.tolist())

    # Save back to JSON
    with open(JSON_PATH, "w") as f:
        json.dump(data, f, indent=4)

    print(f"Base EE saved! Total transforms: {len(data['base_ee'])}")



#capture_image("images1.jpg")
#calculate_T_base_ee(-549.362,42.653,561.389,6.518,54.629,159.229)
#calculate_T_camera_marker("images1.jpg")



    