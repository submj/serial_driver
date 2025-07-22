import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # 参数声明
        self.declare_parameter('port_name', '/dev/ttyCH343USB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1000)  # ms

        # 获取参数
        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout_ms = self.get_parameter('timeout').get_parameter_value().integer_value
        timeout = timeout_ms / 1000.0  # 转秒

        # 打开串口
        try:
            self.serial = serial.Serial(port=port_name, baudrate=baudrate, timeout=timeout)
            self.get_logger().info(f"Opened serial port {port_name} at {baudrate} bps with timeout {timeout}s.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            rclpy.shutdown()
            return

        # 话题发布器 (串口 -> ROS)
        self.publisher_ = self.create_publisher(String, 'serial_rx', 10)

        # 话题订阅器 (ROS -> 串口)
        self.subscription = self.create_subscription(String, 'serial_tx', self.tx_callback, 10)

        # 启动接收线程
        self.rx_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.rx_thread.start()

        # 新增：创建定时器，周期1秒发送字符串“123456”
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            message = "123456"
            self.serial.write((message + '\n').encode('utf-8'))
            self.get_logger().info(f"Timer sent: {message}")
        except Exception as e:
            self.get_logger().error(f"Timer send failed: {e}")

    def tx_callback(self, msg):
        try:
            self.serial.write((msg.data + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Send failed: {e}")

    def read_serial(self):
        while rclpy.ok():
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        msg = String()
                        msg.data = data
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Received: {data}")
            except Exception as e:
                self.get_logger().error(f"Read failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'serial') and node.serial.is_open:
            node.serial.close()
            node.get_logger().info("Serial port closed.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
