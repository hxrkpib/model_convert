#!/bin/bash

# 检查是否提供了文件夹路径
if [ "$#" -ne 1 ]; then
    echo "用法: $0 <文件夹路径>"
    exit 1
fi

# 获取脚本的绝对路径
script_path="$(realpath "$0")"
script_dir="$(dirname "$script_path")"

# 读取文件夹路径
folder_path="$1"

# 获取输入路径的上一层目录
parent_dir="$(dirname "$folder_path")"

# 在上一层目录创建名为 dae 的文件夹
mkdir -p "$parent_dir/dae"

# 在上一层目录创建名为 fbx 的文件夹
mkdir -p "$parent_dir/fbx"

# 在上一层目录创建名为 obj 的文件夹
mkdir -p "$parent_dir/obj"

# 进入 dae 目录
cd "$parent_dir/dae" || { echo "无法进入 dae 目录"; exit 1; }

# 运行同目录下的 Python 脚本，并将文件夹路径作为参数传递
python3 "$script_dir/convert.py" "$folder_path"

