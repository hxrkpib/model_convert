#!/bin/bash

# 读取文件夹路径
urdf_folder_path="/home/lizhen/works/open_source/model_convert/model/urdf"
blender_folder_path="/home/lizhen/blender-4.2.0-linux-x64"

# 获取脚本的绝对路径
script_path="$(realpath "$0")"
script_dir="$(dirname "$script_path")"

# 获取输入路径的上一层目录
parent_dir="$(dirname "$urdf_folder_path")"

# 在上一层目录创建名为 dae 的文件夹
mkdir -p "$parent_dir/dae"

# 在上一层目录创建名为 fbx 的文件夹
mkdir -p "$parent_dir/fbx"

# 在上一层目录创建名为 obj 的文件夹
mkdir -p "$parent_dir/obj"

# 进入 dae 目录
cd "$parent_dir/dae" || { echo "无法进入 dae 目录"; exit 1; }

# 运行同目录下的 Python 脚本，并将文件夹路径作为参数传递
python3 "$script_dir/sdf_dae_convert.py" "$urdf_folder_path"
$blender_folder_path/blender --background --python $script_dir/obj_fbx_convert.py -- "/home/lizhen/works/open_source/model_convert/model/dae/" "$parent_dir/fbx"
