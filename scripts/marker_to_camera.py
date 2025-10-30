#External Library

from pose_estimation import pose_estimation
import cv2
import numpy as np

#Internal Library
from pathlib import Path
import argparse

def marker_to_camera(circles_marker: list[tuple[float, float, float]],input_path: str, output_path: str):
    """
    !Turn a coordinate(in metres) relative to ArUco marker frame to a coordinate(in pixel) relative to camera frame and save the visualization
    @circles_marker: (x,y,z) position (metres) relative to marker frame
    @input_path (str): input image path
    @output_path (str): output image path
    """
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    input_full_path = PROJECT_ROOT / "input" / input_path
    output_full_path = PROJECT_ROOT / "output" / output_path

    INPUT_DIR = PROJECT_ROOT / "input"

    matrix_coefficients_path = INPUT_DIR / "calibration_matrix.npy"
    distortion_coefficients_path = INPUT_DIR / "distortion_coefficients.npy"
    mtx = np.load(matrix_coefficients_path)
    dist = np.load(distortion_coefficients_path)
    image = cv2.imread(input_full_path)

    estimated_frame, rvec, tvec = pose_estimation(
                image=image,
                matrix_coefficients_path=matrix_coefficients_path,
                distortion_coefficients_path=distortion_coefficients_path,
            )

    circles_marker = np.array(circles_marker, dtype=np.float64)

    circle_2d, _ = cv2.projectPoints(circles_marker, rvec, tvec, mtx, dist)

    for pt in circle_2d:
        u, v = int(pt[0][0]), int(pt[0][1])
        cv2.circle(estimated_frame, (u,v), 5, (0,0,255), -1)

    cv2.imwrite(output_full_path, estimated_frame)
def main():
    """
    !construct the argument parse and parse the arguments
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=True,
        help="path to the input image")
    ap.add_argument("-o", "--output", required=True,help="path to output image")
    args = vars(ap.parse_args())
    input_path = args["image"]
    output_path = args["output"]
    circles_marker = [[-0.085, -0.04, 0]]
    marker_to_camera(circles_marker, input_path, output_path)

if __name__ == "__main__":
    main()