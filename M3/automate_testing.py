from nn_config import NNState
from nn_train import Train
from nn_test import Test
from utils import load_yaml
from time import gmtime, strftime
import torch
import subprocess
import csv

if __name__ == "__main__":
    mode = load_yaml("master_config.yaml")["mode"]
    afk_params_dict = {
        "n_dataset": 5,
        "exp_name": "vgg_1",
        "init_lr": [1e-3, 1e-3, 1e-3, 1e-3, 1e-2],
        "batch_size": [32, 64, 128, 256, 256],
        "n_epochs": [10, 15, 20, 25, 20],
        "weight_decay": [1e-5, 1e-5, 1e-5, 1e-5, 1e-4],
        "lr_scheduler": {"step_size": [1, 1, 1, 1, 1],
                         "gamma": [0.9, 0.9, 0.9, 0.9, 0.9]},
        "num_workers": 1,
    }
    params = afk_params_dict
    n_dataset = params["n_dataset"]
    results_file_name = "results.csv"

    if mode == "testing":
        results = []
        # Loop through all different parameters
        for i in range(n_dataset):
            iteration_name = params["exp_name"] + "_" + str(i)
            print(f"\n\n{iteration_name}")
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
                "num_workers": 1,
            }
            test_caller = Test(current_params)
            _, accuracy = test_caller.eval()
            results.append([iteration_name, accuracy])

        # Record results in .csv
        # NOTE: This will only append to csv, NOT rewrite it
        with open(results_file_name, "a") as results_file:
            current_time = strftime("%Y-%m-%d %H:%M:%S", gmtime())
            csv.writer(results_file).writerow([current_time])
            csv.writer(results_file).writerow(["Experiment No.", "Accuracy"])
            for result in results:
                csv.writer(results_file).writerow(result)
        results_file.close()

    elif mode == "training":
        check = input("Shutdown after training?[y/n]\n")
        n_dataset = params["n_dataset"]

        with open(results_file_name, "a") as results_file:
            csv.writer(results_file).writerow(["Training started"])
            current_time = strftime("%Y-%m-%d %H:%M:%S", gmtime())
            csv.writer(results_file).writerow([current_time])
        results_file.close()

        # Loop through all different parameters
        for i in range(n_dataset):
            iteration_name = params["exp_name"] + "_" + str(i)
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
                "num_workers": 1,
            }
            torch.manual_seed(1)
            train_caller = Train(current_params)
            train_caller.train()

        # Record finish time
        with open(results_file_name, "a") as results_file:
            csv.writer(results_file).writerow(["Training finished"])
            current_time = strftime("%Y-%m-%d %H:%M:%S", gmtime())
            csv.writer(results_file).writerow([current_time])
        results_file.close()

        if check.lower() == "y":
            subprocess.call("shutdown")
