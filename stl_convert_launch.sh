#!/bin/bash
# 获取脚本的绝对路径
script_path="$(realpath "$0")"
script_dir="$(dirname "$script_path")"

# 读取文件夹路径
stl_path="/home/lizhen/works/open_source/model_convert/model/urdf/meshes"
convex_hull="/home/lizhen/works/open_source/model_convert/model/urdf/meshes/convex_hull"
cylinders="/home/lizhen/works/open_source/model_convert/model/urdf/meshes/cylinders"
advanced_cylinders="/home/lizhen/works/open_source/model_convert/model/urdf/meshes/advanced_cylinders"
aabb="/home/lizhen/works/open_source/model_convert/model/urdf/meshes/aabb"
obb="/home/lizhen/works/open_source/model_convert/model/urdf/meshes/obb"
spheres="/home/lizhen/works/open_source/model_convert/model/urdf/meshes/spheres"
scale="1.0"
num_samples="1000"
eps="0.05"
min_samples="10"

python3 $script_dir/stl_simplify.py --input $stl_path --output_convex $convex_hull --output_cylinder $cylinders --output_advanced_cylinder $advanced_cylinders --output_aabb $aabb --output_obb $obb --output_spheres $spheres --scale $scale --num_samples $num_samples --eps $eps --min_samples $min_samples
