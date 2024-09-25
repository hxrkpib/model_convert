import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
import csv
import tkinter as tk
import threading
import numpy as np


class PosePublisher(Node):
    def __init__(self, root):
        super().__init__('marker_publisher')
        self.data_path = self.declare_parameter(
            'data_path', '').get_parameter_value().string_value
        self.data_dict = self.read_csv_to_dict(self.data_path)

        link_set = set()
        for key in self.data_dict.keys():
            link_set.add(self.get_part_before_last_two_underscores(key))

        self.get_logger().info(f"set: {link_set}")
        self.link_list = list(link_set)
        self.collision_list = ['right_leg_foot',
                               'left_leg_foot', 'left_hand', 'right_hand']
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/marker_publisher/pose', 10)
        self.plane_publisher = self.create_publisher(
            MarkerArray, '/marker_publisher/plane', 10)

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.marker_array = MarkerArray()
        self.plane_array = MarkerArray()
        self.motion_play = False
        self.motion_iter = 0
        self.timer_iter = 0
        self.freq = 0.03
        self.time_factor = int(self.freq/0.001)
        ######################################## UI ####################################################
        self.root = root
        self.root.title("MOTION EDITOR")

        self.plane_dict = {'default': [
            100, 100, 0, 100, -100, 0, -100, -100, 0, -100, 100, 0]}

        # 创建滑块
        self.slider = tk.Scale(root, from_=0, to=len(self.data_dict[f'{self.link_list[0]}_Position_X'])-1,
                               orient=tk.HORIZONTAL, command=self.update_value)
        self.slider.pack(fill=tk.X, padx=20, pady=10)

        # 创建按钮框
        self.button_frame = tk.Frame(root)
        self.button_frame.pack(pady=10)

        # 创建按钮
        self.play_button = tk.Button(
            self.button_frame, text="Play", command=self.play_action)
        self.play_button.pack(side=tk.LEFT, padx=5)

        self.stop_button = tk.Button(
            self.button_frame, text="Stop", command=self.stop_action)
        self.stop_button.pack(side=tk.LEFT, padx=5)

        self.reset_button = tk.Button(
            self.button_frame, text="Reset", command=self.reset_action)
        self.reset_button.pack(side=tk.LEFT, padx=5)

        # 创建频率标签和输入框
        self.label = tk.Label(self.button_frame, text="Freq:")
        self.label.pack(side=tk.LEFT, padx=5)

        self.entry = tk.Entry(self.button_frame)
        self.entry.pack(side=tk.LEFT, padx=5)
        self.entry.insert(0, str(self.freq))

        # 绑定回车键事件
        self.entry.bind("<Return>", self.on_entry_change)

        # 添加区域
        self.add_frame = tk.Frame(root)
        self.add_frame.pack(pady=10)

        # 添加区域的按钮
        self.button_area = tk.Frame(self.add_frame)
        self.button_area.pack()

        # Add按钮
        self.add_button = tk.Button(
            self.button_area, text="Add", command=self.add_row)
        self.add_button.pack(side=tk.LEFT, padx=5)

        # Print按钮
        self.print_button = tk.Button(
            self.button_area, text="Generate", command=self.generate_csv)
        self.print_button.pack(side=tk.LEFT, padx=5)

        # 区域用于添加行
        self.row_frame = tk.Frame(self.add_frame)
        self.row_frame.pack(pady=5)

        ######################################## UI ####################################################

    def timer_callback(self):
        self.marker_array.markers.clear()
        for index, value in enumerate(self.link_list):
            x, y, z, qx, qy, qz, qw = self.get_pose(
                value, self.motion_iter)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "marker_array"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            collision = False
            if value in self.collision_list:
                for key, value in self.plane_dict.items():
                    if self.is_sphere_intersecting_rectangle([float(x), float(y), float(z)], marker.scale.x/4, [(float(value[0]), float(value[1]), float(value[2])), (float(value[3]), float(value[4]), float(value[5])), (float(value[6]), float(value[7]), float(value[8])), (float(value[9]), float(value[10]), float(value[11]))]):
                        collision = True
                        break
            if collision:
                marker.color.a = 1.0  # Alpha
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0  # Green
                marker.color.b = 0.0  # Blue
            else:
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            self.marker_array.markers.append(marker)

        self.marker_publisher.publish(self.marker_array)
        if self.motion_iter < len(self.data_dict[f'{self.link_list[0]}_Position_X'])-1 and self.motion_play and self.timer_iter % self.time_factor == 0:
            self.motion_iter += 1
            self.slider.set(self.motion_iter)
        self.timer_iter += 1

        # publish plane
        self.plane_array.markers.clear()
        for key, value in self.plane_dict.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "plane_array"
            marker.id = index
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            # marker.color.r = 1.0
            # marker.color.g = 1.0
            # marker.color.b = 0.0
            # marker.color.a = 1.0
            # 定义三角形的顶点
            points = [
                Point(x=float(value[0]), y=float(value[1]),
                      z=float(value[2])),   # 第一个三角形的第一个顶点
                Point(x=float(value[3]), y=float(value[4]),
                      z=float(value[5])),   # 第一个三角形的第二个顶点
                Point(x=float(value[6]), y=float(value[7]),
                      z=float(value[8])),   # 第一个三角形的第三个顶点

                Point(x=float(value[0]), y=float(value[1]),
                      z=float(value[2])),   # 第二个三角形的第一个顶点
                Point(x=float(value[6]), y=float(value[7]),
                      z=float(value[8])),   # 第二个三角形的第二个顶点
                Point(x=float(value[9]), y=float(value[10]),
                      z=float(value[11])),   # 第二个三角形的第三个顶点
            ]
            marker.points = points
            self.plane_array.markers.append(marker)
        self.plane_publisher.publish(self.plane_array)

    def calculate_quaternion_from_points(point_a, point_b):
        # 计算向量
        vector = np.array(point_b) - np.array(point_a)
        direction = vector / np.linalg.norm(vector)  # 归一化

        # 计算旋转四元数
        # 假设 Z 轴为目标方向
        target_direction = np.array([0, 0, 1])

        # 计算旋转轴和角度
        cross_product = np.cross(target_direction, direction)
        dot_product = np.dot(target_direction, direction)

        angle = np.arctan2(np.linalg.norm(cross_product), dot_product)  # 计算夹角
        axis = cross_product / np.linalg.norm(cross_product)  # 归一化旋转轴

        # 使用四元数公式
        qx = axis[0] * np.sin(angle / 2)
        qy = axis[1] * np.sin(angle / 2)
        qz = axis[2] * np.sin(angle / 2)
        qw = np.cos(angle / 2)

        return qx, qy, qz, qw

    def get_pose(self, name, index):
        x = float(self.data_dict[f'{name}_Position_X'][index])/100
        y = float(self.data_dict[f'{name}_Position_Y'][index])/100
        z = float(self.data_dict[f'{name}_Position_Z'][index])/100

        qx = float(self.data_dict[f'{name}_Rotation_X'][index])
        qy = float(self.data_dict[f'{name}_Rotation_Y'][index])
        qz = float(self.data_dict[f'{name}_Rotation_Z'][index])
        qw = float(self.data_dict[f'{name}_Rotation_W'][index])

        return x, y, z, qx, qy, qz, qw

    def get_part_before_last_two_underscores(self, s):
        # 找到最后两个下划线的位置
        first_index = s.rfind('_')
        second_index = s.rfind('_', 0, first_index)

        # 截取字符串
        if second_index != -1:  # 确保找到了第二个下划线
            return s[:second_index]
        return s  # 如果没有找到两个下划线，返回整个字符串

    def read_csv_to_dict(self, file_path):
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

    def is_sphere_intersecting_rectangle(self, sphere_center, radius, points):
        # 将输入转换为 numpy 数组
        C = np.array(sphere_center)
        P1, P2, P3, P4 = map(np.array, points)

        # 计算平面法向量
        A = P2 - P1
        B = P3 - P1
        N = np.cross(A, B)

        # 归一化法向量
        N_normalized = N / np.linalg.norm(N)

        # 计算球心到平面的距离
        d = np.abs(np.dot(N_normalized, C - P1))
        # 检查相交条件
        if d <= radius or C[2] < 0:
            return True
        else:
            return False
        # # 计算球心在平面上的投影点
        # projection = C - d * N_normalized

        # # 检查投影点是否在矩形内
        # def is_point_in_rectangle(point, P1, P2, P3, P4):
        #     def is_point_in_triangle(pt, t1, t2, t3):
        #         # 使用重心坐标法检查点是否在三角形内
        #         area = np.linalg.norm(np.cross(t2 - t1, t3 - t1)) / 2
        #         area1 = np.linalg.norm(np.cross(pt - t1, t2 - t1)) / 2
        #         area2 = np.linalg.norm(np.cross(t2 - t3, pt - t3)) / 2
        #         area3 = np.linalg.norm(np.cross(t3 - t1, pt - t1)) / 2
        #         return np.isclose(area, area1 + area2 + area3)

        #     # 检查点是否在两个三角形内
        #     return (is_point_in_triangle(point, P1, P2, P3) or
        #             is_point_in_triangle(point, P1, P3, P4))

        # return is_point_in_rectangle(projection, P1, P2, P3, P4)
    ######################################## UI ####################################################

    def add_row(self):
        """添加一排包含四个输入框和按钮的行"""
        row = tk.Frame(self.row_frame)
        row.pack(pady=5)

        # 创建每个输入框拆分成3个
        entries = []

        # 添加最左侧的输入框
        left_entry = tk.Entry(row, width=10)  # 单独的输入框
        left_entry.pack(side=tk.LEFT, padx=5)
        entries.append(left_entry)

        for _ in range(4):
            group_frame = tk.Frame(row)
            group_frame.pack(side=tk.LEFT, padx=5)

            for _ in range(3):
                entry = tk.Entry(group_frame, width=10)  # 设置宽度为10字符
                entry.pack(side=tk.LEFT, padx=2)
                entries.append(entry)

            # 添加间隔
            tk.Label(group_frame, text="   ").pack(side=tk.LEFT)

        # 创建Set按钮并绑定打印输入框内容的函数
        set_button = tk.Button(
            row, text="Set", command=lambda: self.row_action(entries))
        set_button.pack(side=tk.LEFT, padx=5)

        # 创建Remove按钮并绑定打印输入框内容的函数
        remove_button = tk.Button(
            row, text="Remove", command=lambda: self.remove_action(entries))
        remove_button.pack(side=tk.LEFT, padx=5)

    def row_action(self, entries):
        values = [entry.get() for entry in entries]
        self.plane_dict[values[0]] = [float(v) for v in values[1:]]

    def remove_action(self, entries):
        values = [entry.get() for entry in entries]
        del self.plane_dict[values[0]]

    def generate_csv(self):
        """打印所有输入框的内容"""
        all_values = []
        for row in self.row_frame.winfo_children():
            for entry in row.winfo_children():
                if isinstance(entry, tk.Entry):
                    all_values.append(entry.get())
        print("All entries:", all_values)

    def on_entry_change(self, event):
        self.freq = float(self.entry.get())
        self.time_factor = int(self.freq/0.001)
        self.timer_iter = 0

    def update_value(self, value):
        self.motion_iter = int(value)
        # self.stop_action()  # Trigger stop action when slider is moved

    def play_action(self):
        self.motion_play = True

    def stop_action(self):
        self.motion_play = False

    def reset_action(self):
        self.slider.set(0)
        self.motion_play = False
        self.motion_iter = 0

    def set_slider_value(self, value):
        """设置滑块的值"""
        self.slider.set(value)
        self.motion_iter = int(value)

    def get_slider_value(self):
        """获取当前滑块的值"""
        return self.slider.get()

    ######################################## UI ####################################################


def run_executor(executor):
    executor.spin()


def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    # 创建节点
    pose_pub_node = PosePublisher(root)

    # 使用MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(pose_pub_node)
    # 启动executor在单独线程
    executor_thread = threading.Thread(
        target=run_executor, args=(executor,), daemon=True)
    executor_thread.start()
    root.mainloop()

    # 关闭节点
    pose_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
