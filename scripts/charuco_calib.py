"""
Sample Usage:-
python charuco_calib.py --dir calibration_charuco/
"""

# Standard Library
import os
import json
import argparse
from typing import Tuple, List
from pathlib import Path

# External Library
import numpy as np
import cv2


def calibrate(
    dirpath: str,
    square_size: float,
    marker_size: float,
    squares_x: int,
    squares_y: int,
    dictionary_id: int = cv2.aruco.DICT_6X6_250,
    visualize: bool = False,
) -> Tuple[float, np.ndarray, np.ndarray, List[np.ndarray], List[np.ndarray]]:
    """! Apply camera calibration operation for images in the given directory
    path using ChArUco board to attain the camera matrix and distortion
    coefficients

    @dirpath (str): Path to the images file for calibration
    @square_size (float): Size of chessboard squares in meters
    @marker_size (float): Size of ArUco markers in meters
    @squares_x (int): Number of squares in X direction
    @squares_y (int): Number of squares in Y direction
    @dictionary_id (int): ArUco dictionary ID
    @visualize (bool): Whether to visualize the calibration process
    """

    # Create ChArUco board
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y), square_size, marker_size, aruco_dict
    )

    # Create detector parameters
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    # Arrays to store points from all images
    all_charuco_corners = []
    all_charuco_ids = []
    image_size = None

    images = os.listdir(dirpath)
    successful_images = 0

    for fname in images:
        img_path = os.path.join(dirpath, fname)
        img = cv2.imread(img_path)

        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]

        # Detect ArUco markers
        marker_corners, marker_ids, rejected = detector.detectMarkers(gray)

        # If at least one marker is detected
        if len(marker_corners) > 0:
            # Interpolate ChArUco corners
            charuco_retval, charuco_corners, charuco_ids = (
                cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, gray, board
                )
            )

            # If enough corners are found, add to calibration data
            if (
                charuco_retval
                and charuco_corners is not None
                and len(charuco_corners) > 3
            ):
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                successful_images += 1

                if visualize:
                    # Draw detected markers and ChArUco corners
                    img_vis = img.copy()
                    cv2.aruco.drawDetectedMarkers(
                        img_vis, marker_corners, marker_ids
                    )
                    cv2.aruco.drawDetectedCornersCharuco(
                        img_vis, charuco_corners, charuco_ids
                    )
                    cv2.imshow("ChArUco Detection", img_vis)
                    cv2.waitKey(500)

    if visualize:
        cv2.destroyAllWindows()

    print(
        "Successfully detected ChArUco board in"
        f" {successful_images}/{len(images)} images"
    )

    if successful_images < 3:
        raise ValueError(
            "Not enough images with detected ChArUco boards for calibration"
        )

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, image_size, None, None
    )

    # Calculate reprojection error
    mean_error = 0
    for i in range(len(all_charuco_corners)):
        reprojected_points, _ = cv2.projectPoints(
            board.getChessboardCorners()[all_charuco_ids[i].flatten()],
            rvecs[i],
            tvecs[i],
            mtx,
            dist,
        )
        error = cv2.norm(
            all_charuco_corners[i], reprojected_points, cv2.NORM_L2
        ) / len(reprojected_points)
        mean_error += error

    print(
        f"Mean reprojection error: {mean_error / len(all_charuco_corners):.6f}"
    )

    return [ret, mtx, dist, rvecs, tvecs]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-d",
        "--dir",
        required=True,
        help="Path to folder containing ChArUco board images for calibration",
    )
    ap.add_argument(
        "-x",
        "--squares_x",
        type=int,
        default=18,
        help="Number of squares in X direction (default=18)",
    )
    ap.add_argument(
        "-y",
        "--squares_y",
        type=int,
        default=13,
        help="Number of squares in Y direction (default=13)",
    )
    ap.add_argument(
        "-s",
        "--square_size",
        type=float,
        default=0.01,
        help="Length of square edge in meters (default=0.01)",
    )
    ap.add_argument(
        "-m",
        "--marker_size",
        type=float,
        default=0.007,
        help="Length of marker edge in meters (default=0.007)",
    )
    ap.add_argument(
        "--dict",
        type=str,
        default="DICT_6X6_250",
        help="ArUco dictionary (default=DICT_6X6_250)",
    )
    ap.add_argument(
        "-v",
        "--visualize",
        action="store_true",
        help="To visualize each ChArUco board detection",
    )
    args = vars(ap.parse_args())

    dirpath = args["dir"]
    square_size = args["square_size"]
    marker_size = args["marker_size"]
    squares_x = args["squares_x"]
    squares_y = args["squares_y"]
    visualize = args["visualize"]

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
    dictionary_id = dict_map.get(args["dict"], cv2.aruco.DICT_6X6_250)

    ret, mtx, dist, rvecs, tvecs = calibrate(
        dirpath,
        square_size,
        marker_size,
        squares_x,
        squares_y,
        dictionary_id,
        visualize,
    )

    print("\nCamera Matrix (K):")
    print(mtx)
    print("\nDistortion Coefficients (D):")
    print(dist)

    # Save to JSON file
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    DATA_DIR = PROJECT_ROOT / "data"
    DATA_DIR.mkdir(exist_ok=True)

    camera_params = {"K": mtx.tolist(), "D": dist.tolist()[0]}

    json_path = DATA_DIR / "camera_params.json"
    with open(json_path, "w") as f:
        json.dump(camera_params, f, indent=4)

    print(f"\nCalibration parameters saved to: {json_path}")
