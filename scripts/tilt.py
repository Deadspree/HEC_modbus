"""
Auto Tilt Correction using ChArUco Board
Computes corrected TCP pose to align camera parallel to board

Sample Usage:-
python tilt.py --image path/to/image.jpg --camera_params data/camera_params.json --robot_ip 192.168.58.2
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Tuple, Optional
from fairino import Robot

import numpy as np
import cv2

# Add parent directory to path for Robot import
sys.path.append(str(Path(__file__).resolve().parent.parent))


def load_camera_params(params_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load camera intrinsic parameters from JSON file

    Args:
        params_path: Path to camera_params.json

    Returns:
        K: Camera matrix (3x3)
        D: Distortion coefficients
    """
    with open(params_path, "r") as f:
        params = json.load(f)

    K = np.array(params["K"], dtype=np.float64)
    D = np.array(params["D"], dtype=np.float64)

    return K, D


def detect_charuco_board(
    image: np.ndarray,
    squares_x: int,
    squares_y: int,
    square_size: float,
    marker_size: float,
    dictionary_id: int = cv2.aruco.DICT_6X6_250,
    visualize: bool = False,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Detect ChArUco board corners in image

    Args:
        image: Input image
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_size: Size of chessboard squares in meters
        marker_size: Size of ArUco markers in meters
        dictionary_id: ArUco dictionary ID
        visualize: Whether to show detection visualization

    Returns:
        charuco_corners: Detected corner coordinates
        charuco_ids: IDs of detected corners
    """
    # Create ChArUco board
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y), square_size, marker_size, aruco_dict
    )

    # Create detector
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    # Convert to grayscale if needed
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    # Detect ArUco markers
    marker_corners, marker_ids, rejected = detector.detectMarkers(gray)

    if len(marker_corners) == 0:
        print("No ArUco markers detected!")
        return None, None

    # Interpolate ChArUco corners
    charuco_retval, charuco_corners, charuco_ids = (
        cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, board
        )
    )

    if charuco_retval == 0 or charuco_corners is None:
        print("ChArUco corners could not be interpolated!")
        return None, None

    print(f"Detected {len(charuco_corners)} ChArUco corners")

    if visualize:
        img_vis = image.copy()
        cv2.aruco.drawDetectedMarkers(img_vis, marker_corners, marker_ids)
        cv2.aruco.drawDetectedCornersCharuco(
            img_vis, charuco_corners, charuco_ids
        )
        cv2.imshow("ChArUco Detection", img_vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return charuco_corners, charuco_ids


def rotation_matrix_to_euler_zyx(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to ZYX Euler angles (Roll-Pitch-Yaw)

    Args:
        R: 3x3 rotation matrix

    Returns:
        Euler angles [roll, pitch, yaw] in radians
    """
    # ZYX order: Rz(yaw) * Ry(pitch) * Rx(roll)
    pitch = np.arcsin(-R[2, 0])

    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))
    else:
        # Gimbal lock case
        roll = 0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    return np.array([roll, pitch, yaw])


def euler_zyx_to_rotation_matrix(
    roll: float, pitch: float, yaw: float
) -> np.ndarray:
    """Convert ZYX Euler angles to rotation matrix

    Args:
        roll, pitch, yaw: Angles in radians

    Returns:
        3x3 rotation matrix
    """
    # Rotation around X (roll)
    Rx = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )

    # Rotation around Y (pitch)
    Ry = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    # Rotation around Z (yaw)
    Rz = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )

    # ZYX order
    R = Rz @ Ry @ Rx
    return R


def compute_corrected_tcp_pose(
    image: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    current_tcp_pose: list,
    squares_x: int,
    squares_y: int,
    square_size: float,
    marker_size: float,
    dictionary_id: int = cv2.aruco.DICT_6X6_250,
    visualize: bool = False,
) -> Optional[list]:
    """Compute corrected TCP pose to make camera parallel to board

    Args:
        image: Input image
        K: Camera matrix
        D: Distortion coefficients
        current_tcp_pose: Current TCP pose [x, y, z, rx, ry, rz]
        (ZYX Euler angles in radians)
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_size: Size of chessboard squares in meters
        marker_size: Size of ArUco markers in meters
        dictionary_id: ArUco dictionary ID
        visualize: Whether to visualize detection

    Returns:
        Corrected TCP pose [x, y, z, rx, ry, rz] or None if failed
    """
    # Detect ChArUco board
    charuco_corners, charuco_ids = detect_charuco_board(
        image,
        squares_x,
        squares_y,
        square_size,
        marker_size,
        dictionary_id,
        visualize,
    )

    if charuco_corners is None or charuco_ids is None:
        print("Failed to detect ChArUco board!")
        return None

    # Create board for pose estimation
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y), square_size, marker_size, aruco_dict
    )

    # Estimate pose of the board relative to camera
    retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board, K, D, None, None
    )

    if not retval:
        print("Failed to estimate board pose!")
        return None

    # Convert rotation vector to rotation matrix
    R_camera_to_board, _ = cv2.Rodrigues(rvec)

    # R_parallel: Rotation needed to align camera Z-axis with board Z-axis
    # (perpendicular to board)
    R_parallel = R_camera_to_board.T

    print("\n" + "=" * 60)
    print("TILT CORRECTION COMPUTATION")
    print("=" * 60)

    print("\nBoard pose relative to camera:")
    print(f"  Translation (tvec): {tvec.flatten()}")
    print(f"  Rotation (rvec): {rvec.flatten()}")

    # Convert current TCP pose to rotation matrix
    current_x, current_y, current_z, current_rx, current_ry, current_rz = (
        current_tcp_pose
    )
    R_current = euler_zyx_to_rotation_matrix(
        current_rx, current_ry, current_rz
    )

    print("\nCurrent TCP pose:")
    print(f"  Position: [{current_x:.4f}, {current_y:.4f}, {current_z:.4f}]")
    print(
        f"  Orientation (rad): [{current_rx:.4f}, {current_ry:.4f},"
        f" {current_rz:.4f}]"
    )
    print(
        f"  Orientation (deg): [{np.degrees(current_rx):.2f},"
        f" {np.degrees(current_ry):.2f}, {np.degrees(current_rz):.2f}]"
    )

    # Compute corrected rotation: R_corrected = R_current * R_parallel
    R_corrected = R_current @ R_parallel

    # Convert back to ZYX Euler angles
    corrected_rx, corrected_ry, corrected_rz = rotation_matrix_to_euler_zyx(
        R_corrected
    )

    # Position stays the same (only orientation changes)
    corrected_pose = [
        current_x,
        current_y,
        current_z,
        corrected_rx,
        corrected_ry,
        corrected_rz,
    ]

    # Calculate correction angles
    correction_angles = rotation_matrix_to_euler_zyx(R_parallel)

    print("\nCorrection needed (ZYX):")
    print(
        f"  Angles (rad): [{correction_angles[0]:.4f},"
        f" {correction_angles[1]:.4f}, {correction_angles[2]:.4f}]"
    )
    print(
        f"  Angles (deg): [{np.degrees(correction_angles[0]):.2f},"
        f" {np.degrees(correction_angles[1]):.2f},"
        f" {np.degrees(correction_angles[2]):.2f}]"
    )

    print("\nCorrected TCP pose:")
    print(
        f"  Position: [{corrected_pose[0]:.4f}, {corrected_pose[1]:.4f},"
        f" {corrected_pose[2]:.4f}]"
    )
    print(
        f"  Orientation (rad): [{corrected_rx:.4f}, {corrected_ry:.4f},"
        f" {corrected_rz:.4f}]"
    )
    print(
        f"  Orientation (deg): [{np.degrees(corrected_rx):.2f},"
        f" {np.degrees(corrected_ry):.2f}, {np.degrees(corrected_rz):.2f}]"
    )

    print("=" * 60)

    return corrected_pose


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Auto tilt correction - compute corrected TCP pose"
    )
    parser.add_argument(
        "-i", "--image", required=True, help="Path to input image"
    )
    parser.add_argument(
        "-c",
        "--camera_params",
        required=True,
        help="Path to camera_params.json",
    )
    parser.add_argument(
        "-r",
        "--robot_ip",
        required=True,
        help="Robot IP address (e.g., 192.168.58.2)",
    )
    parser.add_argument(
        "-x",
        "--squares_x",
        type=int,
        default=18,
        help="Number of squares in X direction (default=18)",
    )
    parser.add_argument(
        "-y",
        "--squares_y",
        type=int,
        default=13,
        help="Number of squares in Y direction (default=13)",
    )
    parser.add_argument(
        "-s",
        "--square_size",
        type=float,
        default=0.01,
        help="Length of square edge in meters (default=0.01)",
    )
    parser.add_argument(
        "-m",
        "--marker_size",
        type=float,
        default=0.007,
        help="Length of marker edge in meters (default=0.007)",
    )
    parser.add_argument(
        "--dict",
        type=str,
        default="DICT_6X6_250",
        help="ArUco dictionary (default=DICT_6X6_250)",
    )
    parser.add_argument(
        "-v", "--visualize", action="store_true", help="Visualize detection"
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Apply correction to robot (move to corrected pose)",
    )

    args = parser.parse_args()

    # Load camera parameters
    K, D = load_camera_params(args.camera_params)
    print("Camera parameters loaded successfully")

    # Load image
    image = cv2.imread(args.image)
    if image is None:
        print(f"Error: Could not load image from {args.image}")
        exit(1)

    # Get ArUco dictionary ID
    dict_map = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    }
    dictionary_id = dict_map.get(args.dict, cv2.aruco.DICT_6X6_250)

    # Get current TCP pose from robot
    print(f"\nConnecting to robot at {args.robot_ip}...")
    try:
        robot = Robot.RPC(args.robot_ip)
        error, pose = robot.GetActualTCPPose()
        robot.CloseRPC()

        if error != 0:
            print(f"Error getting TCP pose: {error}")
            exit(1)

        print(f"Current TCP pose retrieved: {pose}")

    except Exception as e:
        print(f"Error connecting to robot: {e}")
        exit(1)

    # Compute corrected TCP pose
    corrected_pose = compute_corrected_tcp_pose(
        image,
        K,
        D,
        pose,
        args.squares_x,
        args.squares_y,
        args.square_size,
        args.marker_size,
        dictionary_id,
        args.visualize,
    )

    if corrected_pose is None:
        print("\nTilt correction failed!")
        exit(1)

    # Save results
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    DATA_DIR = PROJECT_ROOT / "data"
    DATA_DIR.mkdir(exist_ok=True)

    results = {
        "current_tcp_pose": pose,
        "corrected_tcp_pose": corrected_pose,
        "correction_delta": [corrected_pose[i] - pose[i] for i in range(6)],
    }

    results_path = DATA_DIR / "tcp_correction.json"
    with open(results_path, "w") as f:
        json.dump(results, f, indent=4)

    print(f"\nTCP correction saved to: {results_path}")

    # Apply correction if requested
    if args.apply:
        print("\nApplying correction to robot...")
        try:
            robot = Robot.RPC(args.robot_ip)

            # Move to corrected pose (adjust parameters as needed for your
            # robot)
            joint_pos = robot.GetInverseKin(0, corrected_pose, -1)
            print(f"Moving to corrected joint positions: {joint_pos}")
            error = robot.MoveJ(
                joint_pos,
                tool=1,
                user=0,
                desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                vel=20.0,
                acc=0.0,
                ovl=100.0,
                exaxis_pos=[0.0, 0.0, 0.0, 0.0],
                blendT=-1.0,
                offset_flag=0,
                offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )

            robot.CloseRPC()

            if error == 0:
                print("✓ Robot moved to corrected pose successfully!")
            else:
                print(f"✗ Error moving robot: {error}")

        except Exception as e:
            print(f"✗ Error applying correction: {e}")
    else:
        print("\nTo apply this correction to the robot, run with --apply flag")
        print(
            f"Example: python tilt.py -i {args.image} -c"
            f" {args.camera_params} -r {args.robot_ip} --apply"
        )
