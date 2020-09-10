import subprocess

# Default prefixes for installing apt or pip packages
interpreter = "python3 "

# List of commands to run from terminal
cmd_list = []

# Remove previously trained data
#TODO Load training set from nn_config.yml
file_name = 'pre_trained_01.best.pth.tar'

script_list = ["dataSegmentation.py",
                "nn_train.py",
                "nn_detect.py",
                "nn_test.py"]


for script in script_list:

    cmd_list.append(interpreter + script)

print("List of commands to run")
for a in cmd_list:
    print(a)
print(">>>>>>>>> Running >>>>>>>>>")

for cmd in cmd_list:
    subprocess.call(cmd.split())
print(">>>>>>>>> Done! >>>>>>>>>")
