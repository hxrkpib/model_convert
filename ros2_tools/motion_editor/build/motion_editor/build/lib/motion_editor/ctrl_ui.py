import tkinter as tk


class CtrlUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MOTION EDITOR")

        # 创建滑块
        self.slider = tk.Scale(root, from_=0, to=100,
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
        self.entry.insert(0, "0.01")  # 设置默认值为0.01

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
            self.button_area, text="Print", command=self.print_all_entries)
        self.print_button.pack(side=tk.LEFT, padx=5)

        # 区域用于添加行
        self.row_frame = tk.Frame(self.add_frame)
        self.row_frame.pack(pady=5)

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
        """处理行按钮的点击事件，打印对应输入框的内容"""
        values = [entry.get() for entry in entries]
        print("Set button clicked! Values:", values)

    def remove_action(self, entries):
        """处理Remove按钮的点击事件，打印对应输入框的内容"""
        values = [entry.get() for entry in entries]
        print("Remove button clicked! Values:", values)

    def print_all_entries(self):
        """打印所有输入框的内容"""
        all_values = []
        for row in self.row_frame.winfo_children():
            for entry in row.winfo_children():
                if isinstance(entry, tk.Entry):
                    all_values.append(entry.get())
        print("All entries:", all_values)

    def on_entry_change(self, event):
        print(f"Entry changed: {self.entry.get()}")  # 打印输入框内容

    def update_value(self, value):
        self.stop_action()  # Trigger stop action when slider is moved

    def play_action(self):
        print("Play button clicked!")

    def stop_action(self):
        print("Stop button clicked!")

    def reset_action(self):
        self.slider.set(0)
        print("Reset button clicked!")

    def set_slider_value(self, value):
        """设置滑块的值"""
        self.slider.set(value)

    def get_slider_value(self):
        """获取当前滑块的值"""
        return self.slider.get()


if __name__ == "__main__":
    root = tk.Tk()
    app = CtrlUI(root)

    root.mainloop()
