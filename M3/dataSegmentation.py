# segment the provided / collected images into training, evaluation, and testing set

# import required modules
import json
import os
import shutil
import urllib.request
import pandas as pd

# create folder if doesn't exist
def check_folder(folder):
    if not os.path.exists(folder):
        os.makedirs(folder)

# segment images into training, evaluation, and testing set with balanced classes
def parse_label_file(img_folder, dest_root, label):

    check_folder(dest_root)

    # read in the label file
    csv_file = os.path.join(img_folder, 'labels.csv')
    df = pd.read_csv(csv_file, header = None)
    row_count, _ = df.shape

    # segment by label
    # for i in range(row_count):
    for i in range(500):
        img_label = df[0][i]
        img_name = df[1][i]
        # img_path = os.path.join(img_folder, img_name)
        img_path = os.path.join(img_name)
        segment_path = os.path.join(dest_root, 'All', img_label)
        check_folder(segment_path)
        shutil.copy(img_path, os.path.join(segment_path))

    # group into training, evaluation, and testing sets
    category = label
    all_dir = os.path.join(dest_root, 'All', category)
    trn_dir = os.path.join(dest_root, 'train', category)
    check_folder(trn_dir)
    evl_dir = os.path.join(dest_root, 'eval', category)
    check_folder(evl_dir)
    tst_dir = os.path.join(dest_root, 'test', category)
    check_folder(tst_dir)

    # suggested segmentation ratio: (training, evaluation, testing) = (60%, 20%, 20%)
    f_all = os.listdir(all_dir)
    size = len(f_all)
    trn_size = int(size * 0.6)
    evl_size = int(size * 0.8)
    f_trn = f_all[:trn_size]
    f_evl = f_all[trn_size:evl_size]
    f_tst = f_all[evl_size:]

    for f1 in f_trn:
        shutil.copy(all_dir+f1, trn_dir)
    for f2 in f_evl:
        shutil.copy(all_dir+f2, evl_dir)
    for f3 in f_tst:
        shutil.copy(all_dir+f3, tst_dir)


if __name__ == '__main__':
    objects = ['sheep/', 'coke/', 'neither/']
    base_path = './dataset/'
    for object_name in objects:
        current_path = base_path + object_name
        parse_label_file(current_path, './dataset_test', object_name)
    print("Done")