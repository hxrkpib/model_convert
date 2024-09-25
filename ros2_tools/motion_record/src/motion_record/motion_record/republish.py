import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
import threading

joint_msg = JointState()  # 全局共享的 JointState 消息
lock = threading.Lock()   # 用于保护全局变量的锁

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_republisher')
        self.publisher_ = self.create_publisher(JointState, '/motion_republish/joint_states_serial', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        with lock:
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(joint_msg)

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/ros2_debug/joint_states_serial',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global joint_msg
        # 使用锁来确保线程安全
        with lock:
            joint_msg = msg

def main(args=None):
    rclpy.init(args=args)

    # 创建节点
    joint_state_pub_node = JointStatePublisher()
    joint_state_sub_node = JointStateSubscriber()

    # 使用MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(joint_state_pub_node)
    executor.add_node(joint_state_sub_node)

    try:
        # 使用MultiThreadedExecutor进行多线程执行
        executor.spin()
    except KeyboardInterrupt:
        pass

    # 关闭节点
    joint_state_pub_node.destroy_node()
    joint_state_sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()