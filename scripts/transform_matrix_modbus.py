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


def load_camera_params(
    params_path: str = None,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Load camera intrinsic parameters from JSON file

    Args:
        params_path: Path to camera_params.json (optional, defaults to data/camera_params.json)

    Returns:
        K: Camera matrix (3x3)
        D: Distortion coefficients
    """
    if params_path is None:
        params_path = PROJECT_ROOT / "data" / "camera_params.json"

    with open(params_path, "r") as f:
        params = json.load(f)

    K = np.array(params["K"], dtype=np.float64)
    D = np.array(params["D"], dtype=np.float64)

    return K, D


def new_charuco_pose_estimation(
    image, cameraMatrix, distCoeffs, board, error=True, visualize=False
):
    """
    !Charuco board pose estimation
    @image (np.ndarray): input image
    @cameraMatrix (np.ndarray): camera matrix from calibration
    @distCoeffs (np.ndarray): distortion coefficients from calibration
    @board (cv2.aruco.CharucoBoard): Charuco board object
    """
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    dictionary = board.getDictionary()
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

    marker_corners, marker_ids, rejected = detector.detectMarkers(image)

    charuco_detector = cv2.aruco.CharucoDetector(board)
    (
        charuco_corners,
        charuco_ids,
        charuco_marker_corners,
        charuco_marker_ids,
    ) = charuco_detector.detectBoard(
        image, markerCorners=marker_corners, markerIds=marker_ids
    )
    rvec = None
    tvec = None
    if charuco_ids is not None and len(charuco_corners) > 0:
        objPoints, imgPoints = board.matchImagePoints(
            charuco_corners, charuco_ids
        )

        if len(objPoints) >= 15:
            valid, rvec, tvec = cv2.solvePnP(
                objPoints,
                imgPoints,
                cameraMatrix,
                distCoeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            cv2.solvePnPRefineLM(
                objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec
            )
            proj, _ = cv2.projectPoints(
                objPoints, rvec, tvec, cameraMatrix, distCoeffs
            )
            reproj_err = np.mean(
                np.linalg.norm(proj.squeeze() - imgPoints.squeeze(), axis=1)
            )
            # if reproj_err > 0.5:
            #     return image, None, None
            if error:
                print(f"Reprojection Error: {reproj_err:.4f} pixels")
            if visualize:
                cv2.drawFrameAxes(
                    image, cameraMatrix, distCoeffs, rvec, tvec, 0.03 * 3
                )
                cv2.aruco.drawDetectedCornersCharuco(
                    image, charuco_corners, charuco_ids
                )

                cv2.putText(
                    image,
                    f"Reproj Error: {reproj_err:.3f}px",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 0),
                    2,
                )
                cv2.putText(
                    image,
                    "Adjust the camera so that error is less than 0.3",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

    return image, rvec, tvec


def calculate_T_cam_charuco(
    image: np.ndarray,
    board,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> np.ndarray:
    """
    !Calculate the homogenous transform matrix from camera to charuco
    @image (np.ndarray): input image
    @board (cv2.aruco.CharucoBoard): Charuco board object
    @camera_matrix (np.ndarray): camera matrix from calibration
    @dist_coeffs (np.ndarray): distortion coefficients from calibration
    """
    T_cam_charuco = np.eye(4)

    frame = image
    estimated_frame, rvec, tvec = new_charuco_pose_estimation(
        image=frame,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        board=board,
    )
    R_cm = None
    tvec1 = None
    rvec1 = rvec.flatten()
    tvec1 = tvec.flatten()
    R_cm, _ = cv2.Rodrigues(rvec1)
    T_cam_charuco[:3, :3] = R_cm
    T_cam_charuco[:3, 3] = tvec1
    return T_cam_charuco


def pose_estimation(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_length: float = 0.10,
    aruco_dict_type: int = cv2.aruco.DICT_5X5_100,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    !Perform pose estimation on each frame

    @image (np.ndarray): Input image(BGR format)
    @camera_matrix (np.ndarray): Camera matrix from calibration
    @dist_coeffs (np.ndarray): Distortion coefficients from calibration
    @marker_length (float): Length of marker
    @aruco_dict_type (int): Aruco dictionary type used for detection
    """
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
        obj_points = np.array(
            [
                [0, marker_length, 0],  # TL -> Top-left in image
                [marker_length, marker_length, 0],  # TR
                [marker_length, 0, 0],  # BR
                [0, 0, 0],  # BL -> origin
            ],
            dtype=np.float32,
        )

        for i, corner in enumerate(corners):
            img_points = corner.reshape(4, 2).astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                camera_matrix,
                dist_coeffs,
            )

            if success:
                cv2.drawFrameAxes(
                    image,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    marker_length * 1.5,
                )

                logging.info(
                    f"Marker ID {ids[i][0]} - rvec: {rvec.flatten()}, tvec:"
                    f" {tvec.flatten()}"
                )

    return image, rvec, tvec

def calculate_T_camera_marker(
    image_path, camera_matrix: np.ndarray, dist_coeffs: np.ndarray
) -> np.ndarray:
    """
    !Calculate the Tcamera_marker by using the captured image

    @image_path: input image (np.ndarray or path string)
    @camera_matrix (np.ndarray): Camera matrix from calibration
    @dist_coeffs (np.ndarray): Distortion coefficients from calibration
    """
    T_cam_marker = np.eye(4)
    frame = image_path

    estimated_frame, rvec, tvec = pose_estimation(
        image=frame,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
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

    return T_cam_marker


def calculate_T_base_ee(
    x: float, y: float, z: float, rx: float, ry: float, rz: float
) -> np.ndarray:
    """
    !Calculate T_base_ee from the robot pose

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
    R_x = np.array(
        [
            [1, 0, 0],
            [0, math.cos(rx), -math.sin(rx)],
            [0, math.sin(rx), math.cos(rx)],
        ]
    )

    R_y = np.array(
        [
            [math.cos(ry), 0, math.sin(ry)],
            [0, 1, 0],
            [-math.sin(ry), 0, math.cos(ry)],
        ]
    )

    R_z = np.array(
        [
            [math.cos(rz), -math.sin(rz), 0],
            [math.sin(rz), math.cos(rz), 0],
            [0, 0, 1],
        ]
    )

    # Combined rotation (Z * Y * X)
    R_ee = R_z @ R_y @ R_x
    scale = 0.001  # for metres conversion
    # Homogeneous transformation
    T_base_ee = np.eye(4)
    T_base_ee[:3, :3] = R_ee
    T_base_ee[:3, 3] = [x * scale, y * scale, z * scale]

    return T_base_ee


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
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
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
    # Load camera parameters from JSON
    try:
        camera_matrix, dist_coeffs = load_camera_params()
        print("✅ Camera parameters loaded from data/camera_params.json")
        print(f"Camera Matrix:\n{camera_matrix}")
        print(f"Distortion Coefficients:\n{dist_coeffs}")
    except FileNotFoundError:
        print("❌ Error: data/camera_params.json not found!")
        print(
            "Please run charuco_calib.py first to generate camera calibration."
        )
        return
    except Exception as e:
        print(f"❌ Error loading camera parameters: {e}")
        return

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_EXPOSURE, -2)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 3)
    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera", 960, 540)

    t_cam_marker = None
    t_base_ee = None
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    JSON_PATH = PROJECT_ROOT / "data" / "transforms.json"

    board = cv2.aruco.CharucoBoard(
        (18, 13),
        0.01,
        0.007,
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),
    )

    print("\n📷 Camera Controls:")
    print("   'c' - Capture current pose pair")
    print("   's' - Save pose pair to transforms.json")
    print("   'q' - Quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        in_frame = frame.copy()
        estimated_frame, rvec, tvec = new_charuco_pose_estimation(
            image=in_frame,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            board=board,
            error=False,
            visualize=True,
        )

        cv2.imshow("Camera", estimated_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("c"):
            t_cam_marker = calculate_T_cam_charuco(
                frame, board, camera_matrix, dist_coeffs
            )
            print("t_cam_marker:\n", t_cam_marker)

            try:
                robot = Robot.RPC("192.168.58.2")
                error, pose = robot.GetActualTCPPose()
                t_base_ee = calculate_T_base_ee(
                    pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
                )
                robot.CloseRPC()
                print("t_base_ee:\n", t_base_ee)
            except Exception as e:
                print(f"❌ Error connecting to robot: {e}")

        elif key == ord("s"):
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
            data.append(
                {
                    "T_cam_marker": np.array(t_cam_marker).tolist(),
                    "T_base_ee": np.array(t_base_ee).tolist(),
                }
            )

            # Write back to file
            with open(JSON_PATH, "w") as f:
                json.dump(data, f, indent=4)

            print(f"✅ Saved pair #{len(data)} to {JSON_PATH.name}")

        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
