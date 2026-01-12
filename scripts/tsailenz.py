# Standard Library
import json
import numpy as np
from pathlib import Path

# External Library
import cv2
from scipy.spatial.transform import Rotation


def calibrate_eye_hand(
    T_base_ee_list: list[np.ndarray],
    T_cam_marker_list: list[np.ndarray],
    cal_val: int,
    eye_to_hand: bool = True,
) -> np.ndarray:
    """
    !Function to calculate 4x4 cam to base transform matrix and test matrix
    accuracy
    @T_base_ee_list (list[np.ndarray]): list of T_base_ee matrices
    @T_cam_marker_list (list[np.ndarray]): list of T_cam_marker matrices
    @cal_val (int): Decide numbers of matrixes input pair to calibrate on
    (choose a number that leave the at least 2 matrix pairs of the test)
    , the rest will go to test
    @eye_to_hand (bool): Whether calibration is eye-on-hand or
    eye-to-hand(Default = True)
    """

    R_ee2base = [T[:3, :3] for T in T_base_ee_list]
    t_ee2base = [T[:3, 3].reshape(3, 1) for T in T_base_ee_list]

    R_marker2cam = [T[:3, :3] for T in T_cam_marker_list]
    t_marker2cam = [T[:3, 3].reshape(3, 1) for T in T_cam_marker_list]

    if eye_to_hand:
        # Convert from ee→base to base→ee coordinates
        R_base2ee, t_base2ee, T_ee_base_list = [], [], []

        for R, t in zip(R_ee2base, t_ee2base):
            R_b2g = R.T  # Reshape t_ee2base to (3,1) NOT (3,)
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
            method=cv2.CALIB_HAND_EYE_TSAI,
        )

        T_base_cam = np.eye(4)
        T_base_cam[:3, :3] = R_cam2base
        T_base_cam[:3, 3] = t_cam2base.flatten()
        # T_base_cam[2, 3] = 0.55

        # Test matrix accuracy
        trans_errors = []
        rot_errors = []
        z_errors = []
        x_errors = []
        y_errors = []
        count_below_0_05 = 0  # < 0.05 mm  ( = 0.00005 m )
        count_below_0_1 = 0  # < 0.1 mm   ( = 0.00010 m )
        count_above_0_15 = 0  # > 0.15 mm  ( = 0.00015 m )
        for i in range(len(T_base_ee_list) - 1):
            lhs = T_ee_base_list[i] @ T_base_cam @ T_cam_marker_list[i]
            rhs = T_ee_base_list[i + 1] @ T_base_cam @ T_cam_marker_list[i + 1]

            # Translation error (Euclidean distance)
            t_err = np.linalg.norm(lhs[:3, 3] - rhs[:3, 3])
            # print("t_err:", t_err)
            if t_err < 0.00005:
                count_below_0_05 += 1

            if t_err < 0.00010:
                count_below_0_1 += 1

            if t_err > 0.00015:
                count_above_0_15 += 1
            trans_errors.append(t_err)

            z_error = lhs[2, 3] - rhs[2, 3]
            z_errors.append(z_error)
            y_error = lhs[1, 3] - rhs[1, 3]
            y_errors.append(y_error)
            x_error = lhs[0, 3] - rhs[0, 3]
            x_errors.append(x_error)
            # Rotation error (angle difference in degrees)
            R_err = Rotation.from_matrix(lhs[:3, :3].T @ rhs[:3, :3])
            rot_angle = R_err.magnitude() * 180 / np.pi
            # print("rot_angle", rot_angle)
            rot_errors.append(rot_angle)
        print("Mean translation error (m):", np.mean(trans_errors))
        print("Mean rotation error (deg):", np.mean(rot_errors))
        print("z translation error [m]: ", np.mean(z_errors))
        print("y translation error [m]: ", np.mean(y_errors))
        print("x translation error [m]: ", np.mean(x_errors))
        print("Pairs < 0.05 mm :", count_below_0_05)
        print("Pairs < 0.10 mm :", count_below_0_1)
        print("Pairs > 0.15 mm :", count_above_0_15)

        return T_base_cam
    else:
        # calibrate eye-on-hand
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base=R_ee2base[0:cal_val],
            t_gripper2base=t_ee2base[0:cal_val],
            R_target2cam=R_marker2cam[0:cal_val],
            t_target2cam=t_marker2cam[0:cal_val],
            method=cv2.CALIB_HAND_EYE_TSAI,
        )
        T_gripper_cam = np.eye(4)
        T_gripper_cam[:3, :3] = R_cam2gripper
        T_gripper_cam[:3, 3] = t_cam2gripper.flatten()

        trans_errors = []
        rot_errors = []
        x_errors, y_errors, z_errors = [], [], []

        T_base_marker_list = []
        for i in range(len(T_base_ee_list)):
            T_base_marker = (
                T_base_ee_list[i] @ T_gripper_cam @ T_cam_marker_list[i]
            )
            T_base_marker_list.append(T_base_marker)

        # Calculate mean marker pose in base frame
        mean_translation = np.mean(
            [T[:3, 3] for T in T_base_marker_list], axis=0
        )

        # Error: deviation from mean marker position
        for i, T_base_marker in enumerate(T_base_marker_list):
            # Translation error
            t_err = np.linalg.norm(T_base_marker[:3, 3] - mean_translation)
            trans_errors.append(t_err)

            # Rotation error: compare with first pose as reference
            if i > 0:
                R_err = Rotation.from_matrix(
                    T_base_marker_list[0][:3, :3].T @ T_base_marker[:3, :3]
                )
                rot_angle = R_err.magnitude() * 180 / np.pi
                rot_errors.append(rot_angle)

        # Print only mean errors
        mean_trans_err = np.mean(trans_errors)
        mean_rot_err = np.mean(rot_errors) if rot_errors else 0

        print(
            f"Mean translation error: {mean_trans_err*1000:.4f} mm"
            f" ({mean_trans_err:.6f} m)"
        )
        print(f"Mean rotation error: {mean_rot_err:.4f}°")
        return T_gripper_cam


def main():
    """
    @Load data from the JSON file and run the calibration(eye-to-hand)
    """
    # --- Load the transforms.json file ---
    PROJECT_ROOT = Path(__file__).resolve().parent.parent
    JSON_PATH = PROJECT_ROOT / "data" / "transforms.json"
    with open(JSON_PATH, "r") as f:
        data = json.load(f)
    # Extract homogeneous matrices
    T_base_ee_list = [np.array(item["T_base_ee"]) for item in data]
    T_cam_marker_list = [np.array(item["T_cam_marker"]) for item in data]
    # print("T_base_ee_list_shape: ", T_base_ee_list[0].shape)
    cal_val = len(T_base_ee_list)
    # Get the desired matrix
    T_gripper_cam = calibrate_eye_hand(
        T_base_ee_list, T_cam_marker_list, cal_val, eye_to_hand=False
    )
    print("Samples used: \n", cal_val)
    print("T_base_cam: \n", T_gripper_cam)
    OUTPUT_PATH = PROJECT_ROOT / "data" / "calibrated_matrix.json"
    output_dict = {"T_gripper_cam": T_gripper_cam.tolist()}
    with open(OUTPUT_PATH, "w") as f:
        json.dump(output_dict, f, indent=4)


if __name__ == "__main__":
    main()
