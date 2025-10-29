# Standard Library
import time
import argparse
import logging
from pathlib import Path
import os
import json
# External Library
import cv2
import numpy as np

# Get the project root (assuming this file is somewhere inside the project)
PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Build log file path
log_file = PROJECT_ROOT / "log" / "pose_estimation.log"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_file),
    ]
)

def process_video(input_path: str, output_path: str, realtime: bool) -> None:
     """
     ! Process a video stream frame-by-frame using aruco_deteciton().

     @input_path (str): Path to the input video
     @output_path (str): Path to the output video
     @realtime (bool): Choose to do Tcam-marker in realtime(True)/with video(False)

     """
     T_cam_marker = np.eye(4)
     #For input video to output video
     if not realtime:
        PROJECT_ROOT = Path(__file__).resolve().parent.parent  

        
        

        # Input and output paths relative to project root
        input_full_path = PROJECT_ROOT / "input" / input_path
        output_full_path = PROJECT_ROOT / "output" / output_path

        # Convert to Path objects
        input = Path(input_full_path)
        output = Path(output_full_path)

        has_display = os.environ.get("DISPLAY") is not None or os.name == "nt"

        if not input.exists():
            logging.error(f"Video file not found: {input}")
            raise FileNotFoundError(f"Video file not found: {input}")
        
            cap = cv2.VideoCapture(str(input))
     
        frame_count = 0
        logging.info("Starting video processing... Press 'q' to quit. ")

        out = None
        if output:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            out = cv2.VideoWriter(str(output), fourcc, 30.0, (width, height))
            logging.info(f"Saving processed video to {output}")
        start_time = time.time()

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            INPUT_DIR = PROJECT_ROOT / "input"

            matrix_coefficients_path = INPUT_DIR / "calibration_matrix.npy"
            distortion_coefficients_path = INPUT_DIR / "distortion_coefficients.npy"
            estimated_frame, _, _ = pose_estimation(
                image=frame,
                matrix_coefficients_path=matrix_coefficients_path,
                distortion_coefficients_path=distortion_coefficients_path,
            )
            frame_count += 1

            elapsed_time = time.time() - start_time
            if elapsed_time > 0:
                fps_live = frame_count / elapsed_time
                cv2.putText(estimated_frame, f"FPS: {fps_live:.2f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                            1, (0, 255, 0), 2)
            out.write(estimated_frame)

        # Only show window if we have a display (e.g. running locally)
            if has_display:
                cv2.imshow("Aruco_pose_estimation_realtime", estimated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    logging.info("User pressed 'q'. Exiting video loop.")
                    break

        cap.release()
        if out:
            out.release()

        # Only call this if GUI was used
        if has_display:
            cv2.destroyAllWindows()

    #For realtime      
     else:
         cap = cv2.VideoCapture(1)
         PROJECT_ROOT = Path(__file__).resolve().parent.parent
         while True:
            ret, frame = cap.read()
            if not ret:
                break
            INPUT_DIR = PROJECT_ROOT / "input"
            
            matrix_coefficients_path = INPUT_DIR / "calibration_matrix.npy"
            distortion_coefficients_path = INPUT_DIR / "distortion_coefficients.npy"
            estimated_frame, rvec, tvec = pose_estimation(
                image=frame,
                matrix_coefficients_path=matrix_coefficients_path,
                distortion_coefficients_path=distortion_coefficients_path,
            )

            cv2.imshow("Aruco_pose_estimation_realtime", estimated_frame)
            if cv2.waitKey(1) == ord('q'):    
                break
            if cv2.waitKey(1) == ord('c'):
                rvec = rvec.flatten()
                tvec = tvec.flatten()
                R_cm, _ = cv2.Rodrigues(rvec)
                T_cam_marker[:3, :3] = R_cm
                T_cam_marker[:3, 3] = tvec
                print("T_cam_marker calculated:")
                print(T_cam_marker)
            if cv2.waitKey(1) == ord('s'):
                # file path
                json_path = PROJECT_ROOT / "data" / "transforms.json"
                # Load existing data if file exists
                if os.path.exists(json_path):
                    with open(json_path, "r") as f:
                        data = json.load(f)
                else:
                    data = {"cam_marker": []}  # create empty list if file doesn't exist

                # Append the new pose
                data["cam_marker"].append(T_cam_marker.tolist())

                # Save back to JSON
                with open(json_path, "w") as f:
                    json.dump(data, f, indent=4)

                print(f"Transform saved! Total poses: {len(data['cam_marker'])}")

         cap.release()
         cv2.destroyAllWindows()
            

def pose_estimation(image: np.ndarray,matrix_coefficients_path: (str), distortion_coefficients_path: (str),marker_length: float = 0.07, aruco_dict_type: int = cv2.aruco.DICT_5X5_100) -> np.ndarray:
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

def main():
    """
    !Tcam_marker realtime calculation pipeline

    Usage: pose_estimation.py -rt True

    !Aruco Marker detection with input video

    Usage: pose_estimation.py -i input_video -o output_video

    """
    parser = argparse.ArgumentParser(description="Aruco Marker Detection and Tcam_marker calculation")
    parser.add_argument("-i", "--input", help="Path to input image")
    parser.add_argument("-o", "--output", help="Path to output image")
    parser.add_argument("-rt", "--realtime", default=False, help="True to open" )
    args = vars(parser.parse_args())
   
    input = args["input"]
    output = args["output"]
    realtime = args["realtime"]
    
    process_video(input,output,realtime)




if __name__ == "__main__":
    main()