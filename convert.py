import os
import subprocess
import sys


def convert_urdf_to_sdf(folder_path):
    parent_folder = os.path.dirname(folder_path)
    sdf_folder = os.path.join(parent_folder, 'sdf')
    os.makedirs(sdf_folder, exist_ok=True)

    # 遍历文件夹
    for filename in os.listdir(folder_path):
        if filename.endswith('.urdf'):
            urdf_file_path = os.path.join(folder_path, filename)
            sdf_file_name = filename.replace('.urdf', '.sdf')
            sdf_output_folder = os.path.join(
                sdf_folder, filename.replace('.urdf', ''))
            os.makedirs(sdf_output_folder, exist_ok=True)
            sdf_file_path = os.path.join(sdf_output_folder, sdf_file_name)

            command_sdf = f'gz sdf -p {urdf_file_path} > {sdf_file_path}'
            command_dae = f'gz sim -v 4 -s -r --iterations 1 {sdf_file_path}'
            try:
                subprocess.run(command_sdf, shell=True, check=True)
                print(f'转换成功: {filename} -> {sdf_file_name}')

                template_path = os.path.join(os.path.dirname(
                    __file__), 'templates', 'sdf.templates')
                with open(template_path, 'r') as template_file:
                    template_content = template_file.read()

                with open(sdf_file_path, 'r') as sdf_file:
                    sdf_lines = sdf_file.readlines()

                sdf_content = ''.join(line[1:] for line in sdf_lines[1:-1])
                sdf_content = template_content.replace(
                    '<!-- robot model -->', sdf_content)
                sdf_content = sdf_content.replace(
                    'templates_robot_name', filename.replace('.urdf', ''))

                with open(sdf_file_path, 'w') as sdf_file:
                    sdf_file.write(sdf_content)

                print(f'更新成功: {sdf_file_name} -> {sdf_file_path}')

                subprocess.run(command_dae, shell=True, check=True)
                print(f'dae转换成功: {sdf_file_name}')

            except subprocess.CalledProcessError as e:
                print(f'转换错误: {e}')
            except Exception as e:
                print(f'处理错误: {e}')


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("请提供一个文件夹路径。")
    else:
        folder_path = sys.argv[1]
        convert_urdf_to_sdf(folder_path)
