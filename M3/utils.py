# import os
import csv
import subprocess
import yaml
from typing import Dict
from pathlib import Path
import numpy as np

#TODO Use pandas

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


def load_csv(csv_file_path: str, data_type) -> Dict:
    data = []
    with open(csv_file_path, 'r') as csv_file:
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
