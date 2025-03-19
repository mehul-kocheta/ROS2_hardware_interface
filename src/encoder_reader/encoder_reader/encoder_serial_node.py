import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Adjust as needed
        self.declare_parameter('baud_rate', 115200)
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Connected to {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            return
        
        # ROS2 Publisher
        self.encoder_pub = self.create_publisher(JointState, 'encoder_ticks', 10)
        self.timer = self.create_timer(0.01, self.read_encoder_data)  # 100 Hz

    def read_encoder_data(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                data = json.loads(line)
                
                # Create and publish JointState message
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['left_wheel', 'right_wheel']
                msg.position = [float(data['left_ticks']), float(data['right_ticks'])]
                
                self.encoder_pub.publish(msg)
                
            except (json.JSONDecodeError, KeyError, ValueError) as e:
                self.get_logger().error(f'Error parsing encoder data: {e}')

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

