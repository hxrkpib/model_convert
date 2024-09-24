import bpy
import sys
import os


def enable_exporters():
    """启用 FBX 和 Collada 导出插件，并处理可能的错误。"""
    required_addons = ['io_scene_fbx']
    for addon in required_addons:
        if addon not in bpy.context.preferences.addons:
            try:
                bpy.ops.preferences.addon_enable(module=addon)
                print(f"已启用插件: {addon}")
            except Exception as e:
                print(f"无法启用插件 {addon}: {e}")
                sys.exit(1)
        else:
            print(f"插件已启用: {addon}")


def clear_scene():
    """清除当前场景中的所有对象。"""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    print("已清理当前场景。")


def import_dae(filepath):
    """导入 Collada (.dae) 文件。"""
    try:
        bpy.ops.wm.collada_import(filepath=filepath)
        print(f"成功导入: {filepath}")
        return True
    except Exception as e:
        print(f"导入失败 {filepath}: {e}")
        return False


def export_fbx(filepath):
    """导出为 FBX (.fbx) 文件。"""
    try:
        bpy.ops.export_scene.fbx(filepath=filepath, use_selection=False)
        print(f"成功导出 FBX: {filepath}")
        return True
    except Exception as e:
        print(f"导出 FBX 失败 {filepath}: {e}")
        return False


def convert_file(input_dae, dae_dir, fbx_dir):
    """转换单个 .dae 文件为 .fbx，同时保留目录结构。"""
    try:
        clear_scene()

        # 导入 .dae 文件
        if not import_dae(input_dae):
            print(f"跳过转换: {input_dae}")
            return

        # 获取相对于 dae_dir 的相对路径
        relative_path = os.path.relpath(input_dae, dae_dir)
        # 获取基础文件名（不包括扩展名）
        base_name = os.path.splitext(relative_path)[0]
        # 构建输出 .fbx 文件的完整路径
        fbx_path = os.path.join(fbx_dir, base_name + ".fbx")

        # 确保输出目录存在
        fbx_output_dir = os.path.dirname(fbx_path)
        if not os.path.exists(fbx_output_dir):
            os.makedirs(fbx_output_dir)
            print(f"已创建输出目录: {fbx_output_dir}")

        # 导出为 .fbx
        if export_fbx(fbx_path):
            print(f"已成功转换: {input_dae} -> {fbx_path}")
        else:
            print(f"转换失败: {input_dae}")

    except Exception as e:
        print(f"处理文件 {input_dae} 时发生错误: {e}")


def main():
    # 启用必要的导出插件
    enable_exporters()

    # 解析命令行参数
    argv = sys.argv
    if "--" not in argv:
        argv = []  # 没有额外参数
    else:
        argv = argv[argv.index("--") + 1:]  # 获取 -- 后的参数

    if len(argv) < 2:
        print("用法: blender --background --python obj_fbx_convert.py -- <dae_dir> <fbx_dir>")
        sys.exit(1)

    dae_dir = argv[0]
    fbx_dir = argv[1]

    # 打印输入和输出目录
    print(f"输入 dae 目录: {dae_dir}")
    print(f"输出 fbx 目录: {fbx_dir}")

    # 检查目录是否存在
    if not os.path.isdir(dae_dir):
        print(f"输入 dae 目录不存在: {dae_dir}")
        sys.exit(1)

    if not os.path.isdir(fbx_dir):
        print(f"输出 fbx 目录不存在: {fbx_dir}")
        sys.exit(1)

    print(f"开始转换 .dae 文件从 {dae_dir} 到 {fbx_dir}")

    # 收集所有 .dae 文件
    dae_files = []
    for root, dirs, files in os.walk(dae_dir):
        print(f"遍历目录: {root}")  # 打印当前遍历的目录
        for filename in files:
            print(f"检查文件: {filename}")  # 打印当前检查的文件
            if filename.lower().endswith('.dae'):
                input_dae = os.path.join(root, filename)
                dae_files.append(input_dae)

    print(f"共找到 {len(dae_files)} 个 .dae 文件。")
    print("列出所有找到的 .dae 文件:")
    for dae in dae_files:
        print(f" - {dae}")

    # 遍历并转换每个 .dae 文件
    for idx, input_dae in enumerate(dae_files, start=1):
        print(f"正在处理 ({idx}/{len(dae_files)}): {input_dae}")
        convert_file(input_dae, dae_dir, fbx_dir)

    print("所有文件转换完成。")


if __name__ == "__main__":
    main()
