# Standard library
import json
import numpy as np
from pathlib import Path
import math
import time

# external library
import cv2
from pupil_apriltags import Detector
from fairino import Robot
import ctypes
import tisgrabber as tis

PROJECT_ROOT = Path(__file__).resolve().parent.parent


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
    pose_6dof = np.array([t[0] * 1000, t[1] * 1000, t[2] * 1000, rx, ry, rz])
    return pose_6dof


def compute_safe_top_down_pose(
    T_base_circle: np.ndarray, desired_tool_z=(0, 0, -1.0)
) -> np.ndarray:
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


def extract_mtx(mtx_full_path):
    mtx = np.load(mtx_full_path)
    fx = mtx[0, 0]
    fy = mtx[1, 1]
    cx = mtx[0, 2]
    cy = mtx[1, 2]
    return fx, fy, cx, cy


def april_tag_pose_estimation(
    img: np.ndarray,
    tag_size: int,
    tag_family: str = "tag36h11",
    mtx_path: str = "calibration_matrix.npy",
    dst_path: str = "distortion_coefficients.npy",
):
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    mtx_full_path = PROJECT_ROOT / "input" / mtx_path
    dst_full_path = PROJECT_ROOT / "input" / dst_path
    res = img.copy()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at_detector = Detector(
        families=tag_family,
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )
    fx, fy, cx, cy = extract_mtx(mtx_full_path)

    dst = np.load(dst_full_path)
    tags = at_detector.detect(
        img,
        estimate_tag_pose=True,
        camera_params=[fx, fy, cx, cy],
        tag_size=tag_size,
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
        axis_points = np.float32(
            [
                [0, 0, 0],  # origin
                [axis_len, 0, 0],  # X axis (red)
                [0, axis_len, 0],  # Y axis (green)
                [0, 0, axis_len],  # Z axis (blue)
            ]
        )

        imgpts, _ = cv2.projectPoints(
            axis_points,
            rvec,
            tvec.squeeze(),
            np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]),
            distCoeffs=dst,
        )

        # FIX: convert to integer pixel coords
        imgpts = np.int32(imgpts.reshape(-1, 2))

        # Draw the axes
        cv2.line(
            res, tuple(imgpts[0]), tuple(imgpts[1]), (0, 0, 255), 3
        )  # X - red
        cv2.line(
            res, tuple(imgpts[0]), tuple(imgpts[2]), (0, 255, 0), 3
        )  # Y - green
        cv2.line(
            res, tuple(imgpts[0]), tuple(imgpts[3]), (255, 0, 0), 3
        )  # Z - blue
    return res, rvecs, tvec


def calculate_T_cam_april(image: np.ndarray):
    T_cam_april = np.eye(4)
    _, rvecs, tvec = april_tag_pose_estimation(image, tag_size=0.035)
    T_cam_april[:3, :3] = rvecs
    T_cam_april[:3, 3] = tvec.flatten()
    return T_cam_april


def T_base_marker(
    T_base_cam: np.ndarray,
    T_cam_marker: np.ndarray,
    offset_x: float,
    offset_y: float,
    offset_z: float,
):
    T_base_marker = T_base_cam @ T_cam_marker
    T_base_marker[0, 3] += offset_x  # 0.003533
    T_base_marker[1, 3] += offset_y  # -0.004396
    T_base_marker[2, 3] += offset_z  # 0.0035
    T_base_marker = compute_safe_top_down_pose(T_base_marker)
    position = transform_to_pose6dof_deg(T_base_marker)
    return T_base_marker, position


def calculate_T_base_ee(
    x: float, y: float, z: float, rx: float, ry: float, rz: float
) -> np.ndarray:
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


def main():
    """
    !Pipeline: From taking input image to detect the circle pose in robot base
    frame
    """
    ic = ctypes.cdll.LoadLibrary("./tisgrabber_x64.dll")

    tis.declareFunctions(ic)

    ic.IC_InitLibrary(0)

    hGrabber = tis.openDevice(ic)

    if ic.IC_IsDevValid(hGrabber):
        ic.IC_StartLive(hGrabber, 0)
        while True:
            if ic.IC_SnapImage(hGrabber, 2000) == tis.IC_SUCCESS:

                Width = ctypes.c_long()
                Height = ctypes.c_long()
                BitsPerPixel = ctypes.c_int()
                colorformat = ctypes.c_int()

                ic.IC_GetImageDescription(
                    hGrabber, Width, Height, BitsPerPixel, colorformat
                )

                bpp = BitsPerPixel.value // 8
                buffer_size = Width.value * Height.value * bpp

                imagePtr = ic.IC_GetImagePtr(hGrabber)

                imagedata = ctypes.cast(
                    imagePtr, ctypes.POINTER(ctypes.c_ubyte * buffer_size)
                )

                image = np.ndarray(
                    buffer=imagedata.contents,
                    dtype=np.uint8,
                    shape=(Height.value, Width.value, bpp),
                )
                image = np.ascontiguousarray(image)
                image = cv2.flip(image, 0)
                cv2.namedWindow("TIS Camera", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("TIS Camera", 640, 480)
                cv2.imshow("TIS Camera", image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("c"):
                    cv2.imwrite("capture.jpg", image)
                    T_base_gripper = None
                    robot = Robot.RPC("192.168.58.2")
                    time.sleep(0.05)  # important small delay!
                    error, pose = robot.GetActualTCPPose()
                    T_base_gripper = calculate_T_base_ee(
                        pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
                    )
                    T_cam_marker = calculate_T_cam_april(image)
                    print("T_cam_marker: ", T_cam_marker)
                    # T_cam_circle = calculate_T_cam_circle(
                    # image, x_to_corner = -0.042, y_to_corner = -0.014)
                    # print("T_cam_circle: \n", T_cam_circle)
                    JSON_PATH = (
                        PROJECT_ROOT / "data" / "calibrated_matrix.json"
                    )
                    with open(JSON_PATH, "r") as f:
                        data = json.load(f)
                    T_gripper_cam = np.array(data["T_gripper_cam"])
                    # T_base_circle = T_base_cam @ T_cam_circle
                    # print("T_base_circle: \n", T_base_circle)
                    T_base_marker = (
                        T_base_gripper @ T_gripper_cam @ T_cam_marker
                    )
                    T_base_marker[0, 3] += 0.005188
                    T_base_marker[1, 3] += -0.000718
                    # T_base_marker[2,3] -= 0.093923
                    print("T_base_marker: \n", T_base_marker)
                    T_safe = compute_safe_top_down_pose(T_base_marker)
                    print("T_safe =\n", T_safe)
                    position_place = transform_to_pose6dof_deg(T_safe)
                    position_pick = transform_to_pose6dof_deg(T_safe)
                    position_place = position_pick.copy()
                    position_place[2] += 100  # increase z to pplace
                    print(
                        "position pick: \n",
                        ",".join(map(str, position_pick.flatten())),
                    )
                    print(
                        "position place: \n",
                        ",".join(map(str, position_place.flatten())),
                    )
                    # Calculate circle position in base frame for verification
                    P_circle_marker = np.array([0.05, 0.0, 0.0, 1.0])
                    P_base_circle = T_base_marker @ P_circle_marker
                    P_base_circle = P_base_circle[:3]
                    P_base_circle *= 1000
                    for i in range(3, 6):
                        P_base_circle = np.append(
                            P_base_circle, position_place[i]
                        )
                    print(
                        "circle position in base frame:\n",
                        ",".join(map(str, P_base_circle.flatten())),
                    )
                    robot.CloseRPC()
                if key == ord("q"):
                    break
        ic.IC_StopLive(hGrabber)
        cv2.destroyAllWindows()
    else:
        ic.IC_MsgBox(tis.T("No device opened"), tis.T("Simple Live Video"))

    ic.IC_ReleaseGrabber(hGrabber)


if __name__ == "__main__":
    main()
