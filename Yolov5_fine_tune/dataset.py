'''
Author: Robber swag162534@outlook.com
Date: 2025-03-21 14:36:57
LastEditors: Robber swag162534@outlook.com
LastEditTime: 2025-04-06 10:22:48
FilePath: \MEC202\dataset.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import os
import shutil
import random
from pathlib import Path

def prepare_yolo_dataset(images_folder, annotations_folder, output_folder, split_ratio=0.8):
    """train:val = 8:2"""
    os.makedirs(os.path.join(output_folder, "images", "train"), exist_ok=True)
    os.makedirs(os.path.join(output_folder, "images", "val"), exist_ok=True)
    os.makedirs(os.path.join(output_folder, "labels", "train"), exist_ok=True)
    os.makedirs(os.path.join(output_folder, "labels", "val"), exist_ok=True)


    image_files = [f for f in os.listdir(images_folder) if f.endswith(('.jpg'))]

    random.shuffle(image_files)
    split_idx = int(len(image_files) * split_ratio)
    train_files = image_files[:split_idx]
    val_files = image_files[split_idx:]

    for img_file in train_files:

        shutil.copy(
            os.path.join(images_folder, img_file),
            os.path.join(output_folder, "images", "train", img_file)
        )

        base_name = os.path.splitext(img_file)[0]
        ann_file = f"{base_name}.txt"
        if os.path.exists(os.path.join(annotations_folder, ann_file)):
            shutil.copy(
                os.path.join(annotations_folder, ann_file),
                os.path.join(output_folder, "labels", "train", ann_file)
            )

    for img_file in val_files:

        shutil.copy(
            os.path.join(images_folder, img_file),
            os.path.join(output_folder, "images", "val", img_file)
        )

        base_name = os.path.splitext(img_file)[0]
        ann_file = f"{base_name}.txt"
        if os.path.exists(os.path.join(annotations_folder, ann_file)):
            shutil.copy(
                os.path.join(annotations_folder, ann_file),
                os.path.join(output_folder, "labels", "val", ann_file)
            )

    with open(os.path.join(output_folder, "data.yaml"), "w") as f:
        f.write(f"train: {os.path.join('images', 'train')}\n")
        f.write(f"val: {os.path.join('images', 'val')}\n")
        f.write("nc: 1\n")
        f.write("names: ['golf']\n")

    print(f"training_set: {len(train_files)} images\n")
    print(f"val_set: {len(val_files)} images\n")


images_folder = r"D:\XJTLU\MEC202\videos"
annotations_folder = r"D:\XJTLU\MEC202\extracted_frames\annotations"
output_folder = r"D:\XJTLU\MEC202\yolov11_dataset_videos"
os.makedirs(output_folder, exist_ok=True)
prepare_yolo_dataset(images_folder, annotations_folder, output_folder)
