import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class ArduinoControl(Node):
    def __init__(self):
        super().__init__('arduino_control')

        # Declare and get parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('wheel_radius', 0.055)  # Wheel radius in meters
        self.declare_parameter('wheel_base', 0.4)  # Distance between wheels in meters

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise

        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def apply_dead_zone(self, speed):
        """Adjust speed to avoid values between -0.2 and 0.2, except when zero."""
        if speed == 0.0:
            return 0.0
        elif -0.2 < speed < 0.2:
            return 0.2 if speed > 0 else -0.2
        return speed

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear and angular velocities
        linear_x = msg.linear.x  # Forward/backward speed (m/s)
        angular_z = msg.angular.z  # Rotational speed (rad/s)

        # Convert to wheel speeds (rev/s)
        left_wheel_speed = (linear_x - angular_z * self.wheel_base / 2) / (2 * math.pi * self.wheel_radius)
        right_wheel_speed = (linear_x + angular_z * self.wheel_base / 2) / (2 * math.pi * self.wheel_radius)

        # Clip speeds to the range [-1, 1] for testing
        left_wheel_speed = max(-1.0, min(1.0, left_wheel_speed))
        right_wheel_speed = max(-1.0, min(1.0, right_wheel_speed))

        # Apply dead zone correction
        left_wheel_speed = self.apply_dead_zone(left_wheel_speed)
        right_wheel_speed = self.apply_dead_zone(right_wheel_speed)

        # Send the motor speeds as a serial message
        command = f"{left_wheel_speed:.2f},{right_wheel_speed:.2f}\n"
        self.get_logger().info(f"Sending to Arduino: {command.strip()}")
        try:
            self.serial_conn.write(command.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send data to Arduino: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        # Clean up
        node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()