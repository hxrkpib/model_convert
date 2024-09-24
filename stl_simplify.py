import os
import trimesh
import argparse
import logging
from tqdm import tqdm
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.optimize import least_squares


def create_convex_hull(mesh, scale_factor=1.0):
    """
    生成网格的凸包，并应用缩放比例。

    :param mesh: trimesh 的 Mesh 对象
    :param scale_factor: 缩放比例
    :return: 缩放后的凸包 Mesh 对象
    """
    convex_hull = mesh.convex_hull
    convex_hull.apply_scale(scale_factor)
    return convex_hull


def create_cylinder_from_mesh(mesh, scale_factor=1.0):
    """
    根据给定网格的定向边界框（OBB）创建一个内接的圆柱体，并应用缩放比例。

    :param mesh: trimesh 的 Mesh 对象
    :param scale_factor: 缩放比例
    :return: trimesh 的 Cylinder 对象
    """
    # 获取网格的定向边界框（OBB）
    obb = mesh.bounding_box_oriented
    obb_transform = obb.primitive.transform  # 4x4 变换矩阵
    obb_extents = obb.primitive.extents  # [长度, 宽度, 高度]

    # 确定主轴（假设主轴是最长的边界框维度）
    main_axis_index = np.argmax(obb_extents)
    main_axis_length = obb_extents[main_axis_index]
    # 其他两个维度用于计算半径
    cross_extents = np.delete(obb_extents, main_axis_index)
    # 半径为其他两个维度中较小的那一个的一半，以确保圆柱体内接
    radius = min(cross_extents) / 2

    # 应用缩放比例
    height = main_axis_length * scale_factor
    radius *= scale_factor

    # 创建一个沿 Z 轴的圆柱体
    cylinder = trimesh.creation.cylinder(
        radius=radius, height=height, sections=32)

    # 获取 OBB 的主轴方向
    # OBB 的变换矩阵的列向量表示 OBB 的三个局部轴
    main_axis_direction = obb_transform[:3, main_axis_index]

    # 计算将 Z 轴对齐到主轴方向的旋转矩阵
    z_axis = np.array([0, 0, 1])
    if np.allclose(main_axis_direction, z_axis):
        rotation_matrix = np.eye(4)
    elif np.allclose(main_axis_direction, -z_axis):
        # 如果主轴方向与 Z 轴相反，旋转 180 度
        rotation_matrix = trimesh.transformations.rotation_matrix(np.pi, [
                                                                  1, 0, 0])
    else:
        rotation_matrix = trimesh.geometry.align_vectors(
            z_axis, main_axis_direction)

    # 应用旋转
    cylinder.apply_transform(rotation_matrix)

    # 获取 OBB 的中心位置
    obb_center = obb.centroid * scale_factor

    # 将圆柱体移动到 OBB 的中心位置
    cylinder.apply_translation(obb_center - cylinder.centroid)

    return cylinder


def create_aligned_cylinder_from_mesh(mesh, scale_factor=1.0):
    """
    根据给定网格的主轴对齐创建一个适配的圆柱体。

    :param mesh: trimesh 的 Mesh 对象
    :param scale_factor: 缩放比例
    :return: trimesh 的 Cylinder 对象
    """
    principal_axes = mesh.principal_inertia_vectors
    if principal_axes.shape[1] < 1:
        logging.warning("网格的主轴数量不足，无法进行高级圆柱体拟合。将使用默认对齐。")
        return create_cylinder_from_mesh(mesh, scale_factor)

    # 选择最大的惯性向量作为主轴
    inertia_lengths = np.linalg.norm(principal_axes, axis=0)
    principal_axis = principal_axes[:, np.argmax(inertia_lengths)]

    # 获取网格的边界框
    bounds = mesh.bounds
    min_bound = bounds[0]
    max_bound = bounds[1]

    # 计算高度（沿主轴方向）
    height = max_bound[2] - min_bound[2]

    # 计算半径（以 X 和 Y 方向的最大值为基础）
    radius = max(max_bound[0] - min_bound[0], max_bound[1] - min_bound[1]) / 2

    # 应用缩放比例
    height *= scale_factor
    radius *= scale_factor

    # 创建圆柱体，默认沿 Z 轴
    cylinder = trimesh.creation.cylinder(
        radius=radius, height=height, sections=32)

    # 计算旋转矩阵，将 Z 轴对齐到主轴
    z_axis = np.array([0, 0, 1])
    if np.allclose(principal_axis, z_axis) or np.allclose(principal_axis, -z_axis):
        rotation_matrix = np.eye(4)
    else:
        rotation_matrix = trimesh.geometry.align_vectors(
            z_axis, principal_axis)

    # 应用旋转
    cylinder.apply_transform(rotation_matrix)

    # 将圆柱体移动到网格的中心位置
    mesh_center = mesh.centroid
    cylinder.apply_translation(mesh_center - cylinder.centroid)

    return cylinder


def create_aabb(mesh, scale_factor=1.0):
    """
    生成网格的轴对齐边界框（AABB），并应用缩放比例。

    :param mesh: trimesh 的 Mesh 对象
    :param scale_factor: 缩放比例
    :return: 轴对齐边界框的 Mesh 对象
    """
    aabb = mesh.bounding_box
    aabb.apply_scale(scale_factor)
    return aabb


def create_obb(mesh, scale_factor=1.0):
    """
    生成网格的定向边界框（OBB），并应用缩放比例。

    :param mesh: trimesh 的 Mesh 对象
    :param scale_factor: 缩放比例
    :return: 定向边界框的 Mesh 对象
    """
    obb = mesh.bounding_box_oriented
    obb.apply_scale(scale_factor)
    return obb


def fit_spheres(mesh, scale_factor=1.0, num_samples=1000, eps=0.05, min_samples=10):
    """
    对网格进行多个球体拟合。

    :param mesh: trimesh 的 Mesh 对象
    :param scale_factor: 缩放比例
    :param num_samples: 从网格上采样的点数
    :param eps: DBSCAN 聚类的邻域半径
    :param min_samples: DBSCAN 聚类的最小样本数
    :return: 拟合的球体列表，每个球体为 (center, radius) 的元组
    """
    # 从网格上采样点
    points, _ = trimesh.sample.sample_surface(mesh, num_samples)

    # 使用 DBSCAN 进行聚类
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = clustering.labels_

    unique_labels = set(labels)
    unique_labels.discard(-1)  # 移除噪声点

    spheres = []

    for label in unique_labels:
        cluster_points = points[labels == label]
        if len(cluster_points) < 4:
            continue  # 需要至少4个点才能拟合球体

        # 初始猜测
        x0 = np.mean(cluster_points, axis=0)
        r0 = np.mean(np.linalg.norm(cluster_points - x0, axis=1))

        # 最小二乘法拟合球体
        def residuals(params, points):
            center = params[:3]
            radius = params[3]
            return np.linalg.norm(points - center, axis=1) - radius

        initial_guess = np.hstack((x0, r0))
        result = least_squares(residuals, initial_guess,
                               args=(cluster_points,))

        if result.success:
            fitted_center = result.x[:3] * scale_factor
            fitted_radius = result.x[3] * scale_factor
            spheres.append((fitted_center, fitted_radius))

    return spheres


def create_spheres_mesh(spheres):
    """
    根据拟合的球体列表创建一个合并后的球体 Mesh 对象。

    :param spheres: 拟合的球体列表，每个球体为 (center, radius) 的元组
    :return: 合并后的球体 Mesh 对象
    """
    sphere_meshes = []
    for center, radius in spheres:
        sphere = trimesh.creation.icosphere(subdivisions=3, radius=radius)
        sphere.apply_translation(center)
        sphere_meshes.append(sphere)

    if not sphere_meshes:
        return None

    combined = trimesh.util.concatenate(sphere_meshes)
    return combined


def setup_logging():
    """
    设置日志格式和级别。
    """
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s')


def parse_arguments():
    """
    解析命令行参数。

    :return: 解析后的参数对象
    """
    parser = argparse.ArgumentParser(
        description='为 STL 文件生成多种拟合模型，并保存到不同的输出文件夹。')
    parser.add_argument('--input', type=str,
                        default='stl_files', help='包含原始 STL 文件的输入文件夹。')
    parser.add_argument('--output_convex', type=str,
                        default='convex_hull', help='保存凸包 STL 文件的输出文件夹。')
    parser.add_argument('--output_cylinder', type=str,
                        default='cylinders', help='保存圆柱体 STL 文件的输出文件夹。')
    parser.add_argument('--output_advanced_cylinder', type=str,
                        default='advanced_cylinders', help='保存高级圆柱体拟合 STL 文件的输出文件夹。')
    parser.add_argument('--output_aabb', type=str,
                        default='aabb', help='保存轴对齐边界框 STL 文件的输出文件夹。')
    parser.add_argument('--output_obb', type=str,
                        default='obb', help='保存定向边界框 STL 文件的输出文件夹。')
    parser.add_argument('--output_spheres', type=str,
                        default='spheres', help='保存球体拟合 STL 文件的输出文件夹。')
    parser.add_argument('--scale', type=float, default=1.0, help='生成模型的缩放比例。')
    parser.add_argument('--num_samples', type=int,
                        default=1000, help='用于球体拟合的点云采样数。')
    parser.add_argument('--eps', type=float, default=0.05,
                        help='DBSCAN 聚类的邻域半径。')
    parser.add_argument('--min_samples', type=int,
                        default=10, help='DBSCAN 聚类的最小样本数。')
    return parser.parse_args()


def ensure_directory(directory):
    """
    确保指定的目录存在，如果不存在则创建。

    :param directory: 目录路径
    """
    if not os.path.exists(directory):
        os.makedirs(directory)
        logging.info(f"已创建目录：{directory}")


def process_stl_files(input_folder, output_convex, output_cylinder, output_advanced_cylinder, output_aabb, output_obb, output_spheres, scale_factor, num_samples, eps, min_samples):
    """
    处理输入文件夹中的所有 STL 文件，生成各种拟合模型，并保存到相应的输出文件夹。

    :param input_folder: 输入文件夹路径
    :param output_convex: 保存凸包的输出文件夹路径
    :param output_cylinder: 保存圆柱体的输出文件夹路径
    :param output_advanced_cylinder: 保存高级圆柱体拟合的输出文件夹路径
    :param output_aabb: 保存轴对齐边界框的输出文件夹路径
    :param output_obb: 保存定向边界框的输出文件夹路径
    :param output_spheres: 保存球体拟合的输出文件夹路径
    :param scale_factor: 缩放比例
    :param num_samples: 点云采样数
    :param eps: DBSCAN 聚类的邻域半径
    :param min_samples: DBSCAN 聚类的最小样本数
    """
    stl_files = [f for f in os.listdir(
        input_folder) if f.lower().endswith('.stl')]

    if not stl_files:
        logging.warning(f"在 '{input_folder}' 中未找到任何 STL 文件。")
        return

    for filename in tqdm(stl_files, desc="处理 STL 文件"):
        file_path = os.path.join(input_folder, filename)
        try:
            mesh = trimesh.load_mesh(file_path)
            if not isinstance(mesh, trimesh.Trimesh):
                logging.warning(
                    f"文件 '{filename}' 未加载为 Trimesh 对象，可能包含多个网格。将尝试处理第一个网格。")
                if isinstance(mesh, trimesh.Scene):
                    if len(mesh.geometry) == 0:
                        logging.error(f"文件 '{filename}' 中不包含有效的几何体。")
                        continue
                    mesh = list(mesh.geometry.values())[0]
                else:
                    logging.error(f"无法处理文件 '{filename}'。")
                    continue

            if not mesh.is_watertight:
                logging.warning(f"网格 '{filename}' 不是水密的。生成的模型可能不准确。")

            # 生成并保存凸包
            convex_hull = create_convex_hull(mesh, scale_factor)
            convex_output_path = os.path.join(output_convex, filename)
            convex_hull.export(convex_output_path)
            logging.info(f"已生成凸包并保存到 '{convex_output_path}'。")

            # 生成并保存圆柱体
            cylinder = create_cylinder_from_mesh(mesh, scale_factor)
            cylinder_output_path = os.path.join(output_cylinder, filename)
            cylinder.export(cylinder_output_path)
            logging.info(f"已生成圆柱体并保存到 '{cylinder_output_path}'。")

            # # 生成并保存高级圆柱体拟合
            # advanced_cylinder = create_aligned_cylinder_from_mesh(
            #     mesh, scale_factor)
            # advanced_cylinder_output_path = os.path.join(
            #     output_advanced_cylinder, filename)
            # advanced_cylinder.export(advanced_cylinder_output_path)
            # logging.info(f"已生成高级圆柱体拟合并保存到 '{advanced_cylinder_output_path}'。")

            # # 生成并保存轴对齐边界框（AABB）
            # aabb = create_aabb(mesh, scale_factor)
            # aabb_output_path = os.path.join(output_aabb, filename)
            # aabb.export(aabb_output_path)
            # logging.info(f"已生成轴对齐边界框并保存到 '{aabb_output_path}'。")

            # 生成并保存定向边界框（OBB）
            obb = create_obb(mesh, scale_factor)
            obb_output_path = os.path.join(output_obb, filename)
            obb.export(obb_output_path)
            logging.info(f"已生成定向边界框并保存到 '{obb_output_path}'。")

            # 生成并保存多个球体拟合
            spheres = fit_spheres(mesh, scale_factor,
                                  num_samples, eps, min_samples)
            if spheres:
                spheres_mesh = create_spheres_mesh(spheres)
                if spheres_mesh:
                    spheres_output_path = os.path.join(
                        output_spheres, f"spheres_{os.path.splitext(filename)[0]}.stl")
                    spheres_mesh.export(spheres_output_path)
                    logging.info(f"已生成多个球体拟合并保存到 '{spheres_output_path}'。")
                else:
                    logging.warning(f"未生成任何球体拟合模型对于文件 '{filename}'。")
            else:
                logging.warning(f"未找到适合拟合的球体对于文件 '{filename}'。")

        except Exception as e:
            logging.error(f"处理文件 '{filename}' 时出错：{e}")


def main():
    """
    主函数，执行脚本的主要逻辑。
    """
    setup_logging()
    args = parse_arguments()

    input_folder = args.input
    output_convex = args.output_convex
    output_cylinder = args.output_cylinder
    output_advanced_cylinder = args.output_advanced_cylinder
    output_aabb = args.output_aabb
    output_obb = args.output_obb
    output_spheres = args.output_spheres
    scale_factor = args.scale
    num_samples = args.num_samples
    eps = args.eps
    min_samples = args.min_samples

    # 确保所有输出目录存在
    ensure_directory(output_convex)
    ensure_directory(output_cylinder)
    ensure_directory(output_advanced_cylinder)
    ensure_directory(output_aabb)
    ensure_directory(output_obb)
    ensure_directory(output_spheres)

    # 处理 STL 文件
    process_stl_files(input_folder, output_convex, output_cylinder, output_advanced_cylinder,
                      output_aabb, output_obb, output_spheres, scale_factor, num_samples, eps, min_samples)

    logging.info("所有模型已生成并保存。")


if __name__ == "__main__":
    main()
