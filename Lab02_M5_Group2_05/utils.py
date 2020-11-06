import csv
import subprocess
import yaml
from typing import Dict
from pathlib import Path
import numpy as np


def load_yaml(cfg_file_path: str) -> Dict:
    """Load yaml file
    Args:
        cfg_file_path (str): .yaml file path
    Returns:
        Dict: All yaml file data
    """
    with open(cfg_file_path, "r") as cfg_file:
        try:
            cfg = yaml.load(cfg_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return cfg


def load_calib_params(file_path: str = None):
    """See absolute file paths for file structure
    Args:
        file_path (str): Folder containing calibration subfolders
    """
    # Imports camera / wheel calibration parameters
    fileK = f"{file_path}camera_calibration/intrinsic.txt"
    camera_matrix = np.loadtxt(fileK, delimiter=",")
    fileD = f"{file_path}camera_calibration/distCoeffs.txt"
    distortion_coeffs = np.loadtxt(fileD, delimiter=",")
    fileS = f"{file_path}wheel_calibration/scale.txt"
    scale = np.loadtxt(fileS, delimiter=",")
    fileB = f"{file_path}wheel_calibration/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=",")
    return camera_matrix, distortion_coeffs, scale, baseline


def wrap_angle(yaw, deg=False):
    """Wrap angle between -pi and pi
    Args:
        yaw (np): Angle in radians.
    Returns:
        [type]: [description]
    """
    yaw = yaw % np.pi
    if yaw < 0:
        yaw += 2 * np.pi
    if deg:
        yaw = np.rad2deg(yaw)
    return yaw


def load_csv(csv_file_path: str, data_type) -> Dict:
    data = []
    with open(csv_file_path, "r") as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            data.append(row)
        np_data = np.asfarray(data, dtype=data_type)
    csv_file.close()
    return np_data


def check_folder(folder_path: Path, create_folder: bool = False) -> None:
    """Check if folder exists
    Args:
        folder_path (Path): Path to folder
        create_folder (bool, optional): Create folder if it doesn't exist. Defaults to False.
    """
    if not folder_path.exists():
        if create_folder:
            folder_path.mkdir(parents=True, exist_ok=True)
            print(f"Created {folder_path}")


def check_file(file_path: Path, delete: bool = False) -> bool:
    """Check if file exists
    Args:
        file_path (Path): Path to file
        delete (bool, optional): Delete file if it exists. Defaults to False.
    """
    if file_path.exists():
        result = True
        if delete:
            file_path.unlink()
            result = False
            print(f"Emptied contents of {file_path}")
        return result


def delete_tar(file_name: str) -> None:
    """Delete weights .tar file from training if name hasn't been changed
    Args:
        file_name (str): [description]
    """
    weights_path = Path.cwd() + "net_weights" + file_name
    if weights_path.exists() and weights_path.is_file():
        subprocess.call("rm *.tar")
