from nn_config import NNState
from nn_train import Train
from nn_test import Test
from utils import load_yaml
import torch

if __name__ == "__main__":
    master_cfg = load_yaml("master_config.yaml")
    mode = master_cfg["mode"]
    afk_params_dict = {
        "n_dataset": 3,
        "exp_name": "trained_1",
        "init_lr": [1.0e-3, 2e-3, 3e-3],
        "batch_size": [128, 128, 128],
        "n_epochs": [15, 15, 15],
        "weight_decay": [1.0e-5, 1e-5, 1e-5],
        "lr_scheduler": {"step_size": [1, 1, 1], "gamma": [0.9, 0.9, 0.9]},
    }
    params = afk_params_dict
    n_dataset = params["n_dataset"]

    if mode == "testing":
        results = []
        for i in range(n_dataset):
            iteration_name = params["exp_name"] + "_" +str(i)
            test_caller = Test()
            results.append([iteration_name, test_caller.eval()])

    elif mode == "training":
        params = afk_params_dict
        n_dataset = afk_params_dict["n_dataset"]
        for i in range(n_dataset):
            iteration_name = params["exp_name"] + "_" +str(i)
            current_params = {
                "exp_name": iteration_name,
                "init_lr": params["init_lr"][i],
                "batch_size": params["batch_size"][i],
                "n_epochs": params["n_epochs"][i],
                "weight_decay": params["weight_decay"][i],
                "lr_scheduler": {
                    "step_size": params["lr_scheduler"]["step_size"][i],
                    "gamma": params["lr_scheduler"]["gamma"][i],
                },
            }
            torch.manual_seed(1)
            train_caller = Train(current_params)
            train_caller.train()
