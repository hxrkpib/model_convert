import bpy
import os

path = "/home/lizhen/fbx"
# 指定起始帧和结束帧
start_frame = 1  # 替换为你的起始帧
end_frame = 10   # 替换为你的结束帧
# 指定要输出的骨骼名称列表
selected_bones = [("mixamorig:Hips", "body")]  # 替换为你要输出的骨骼名称


# 设置输出文件路径
output_file_path = os.path.join(
    bpy.path.abspath(path), "pose_data.csv")

# 检查并创建输出目录
output_dir = os.path.dirname(output_file_path)
if not os.path.exists(output_dir):
    os.makedirs(output_dir)


# 获取当前场景的帧率 (FPS)
fps = bpy.context.scene.render.fps

# 获取现有骨骼名称
existing_bones = bpy.context.object.pose.bones.keys()
print(f"Existing bones: {existing_bones}")

# 检查并打印不存在的骨骼
for bone_name in selected_bones:
    if bone_name[0] not in existing_bones:
        print(
            f"Warning: Bone '{bone_name[0]}' does not exist in the armature.")

# 打开文件以写入位姿数据
with open(output_file_path, 'w') as f:
    # 写入标题行
    header = [f"{bone[1]}_Position_X,{bone[1]}_Position_Y,{bone[1]}_Position_Z,{bone[1]}_Rotation_X,{bone[1]}_Rotation_Y,{bone[1]}_Rotation_Z,{bone[1]}_Rotation_W" for bone in selected_bones]
    f.write(",".join(header) + "\n")

    # 写入帧率行
    f.write(f"{fps}\n")

    # 遍历每一帧
    for frame in range(start_frame, end_frame + 1):
        bpy.context.scene.frame_set(frame)  # 设置当前帧
        print(f"Setting frame: {frame}")  # 调试信息

        row = []
        for bone_name in selected_bones:
            if bone_name[0] in existing_bones:
                bone = bpy.context.object.pose.bones[bone_name[0]]
                bone_matrix = bone.matrix
                world_position = bone_matrix.to_translation()
                world_rotation = bone_matrix.to_quaternion()

                row.extend([world_position.x, world_position.y, world_position.z,
                            world_rotation.x, world_rotation.y, world_rotation.z, world_rotation.w])
            else:
                # 额外的调试信息
                print(f"Bone '{bone_name[0]}' is not in the armature.")

        if len(row) > 1:  # 只有在 row 有数据时才写入
            f.write(",".join(map(str, row)) + "\n")

print(f"Pose data exported to: {output_file_path}")
