import tkinter as tk


class TimeAxisApp:
    def __init__(self, master):
        self.master = master
        self.master.title("时间轴选择")

        self.canvas = tk.Canvas(master, width=600, height=100, bg="white")
        self.canvas.pack()

        # 时间轴的起始和结束点
        self.start_point = None
        self.end_point = None
        self.dragging = None  # 记录当前拖动的是哪个点

        self.draw_time_axis()

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

    def draw_time_axis(self):
        self.canvas.delete("all")
        self.canvas.create_line(50, 50, 550, 50, fill="black", width=2)

        # 绘制起始和结束点
        if self.start_point is not None:
            self.canvas.create_line(
                self.start_point, 40, self.start_point, 60, fill="red", width=3)
        if self.end_point is not None:
            self.canvas.create_line(
                self.end_point, 40, self.end_point, 60, fill="green", width=3)

    def on_click(self, event):
        if self.start_point is None:
            self.start_point = event.x
            self.dragging = 'start'
        elif self.end_point is None:
            self.end_point = event.x
            self.dragging = 'end'
        else:
            # 如果两个点都存在，判断点击哪个点
            if abs(event.x - self.start_point) < abs(event.x - self.end_point):
                self.dragging = 'start'
            else:
                self.dragging = 'end'

        self.draw_time_axis()

    def on_drag(self, event):
        if self.dragging == 'start':
            self.start_point = max(
                50, min(event.x, self.end_point if self.end_point is not None else 550))
        elif self.dragging == 'end':
            self.end_point = min(
                550, max(event.x, self.start_point if self.start_point is not None else 50))
        self.draw_time_axis()

    def on_release(self, event):
        if self.start_point is not None and self.end_point is not None:
            print(f"选择的时间范围: {self.start_point} - {self.end_point}")
            self.dragging = None


if __name__ == "__main__":
    root = tk.Tk()
    app = TimeAxisApp(root)
    root.mainloop()
