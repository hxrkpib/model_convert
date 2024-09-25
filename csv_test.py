import csv


def get_part_before_last_two_underscores(s):
    # 找到最后两个下划线的位置
    first_index = s.rfind('_')
    second_index = s.rfind('_', 0, first_index)

    # 截取字符串
    if second_index != -1:  # 确保找到了第二个下划线
        return s[:second_index]
    return s  # 如果没有找到两个下划线，返回整个字符串


def read_csv_to_dict(file_path):
    data_dict = {}

    with open(file_path, mode='r', encoding='utf-8') as file:
        csv_reader = csv.reader(file)
        headers = next(csv_reader)  # 读取表头

        # 初始化字典，键为表头，值为空列表
        for header in headers:
            data_dict[header] = []

        # 将每一列数据添加到相应的列表中
        for row in csv_reader:
            for header, value in zip(headers, row):
                data_dict[header].append(value)

    return data_dict


# 使用示例
file_path = '/home/lizhen/fbx/pose_data.csv'
result = read_csv_to_dict(file_path)

link_set = set()
for key in result.keys():
    link_set.add(get_part_before_last_two_underscores(key))
# 打印结果
print(link_set)
