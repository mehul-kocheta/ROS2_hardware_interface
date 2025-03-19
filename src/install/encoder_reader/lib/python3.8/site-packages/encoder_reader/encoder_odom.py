import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odom')

        # Subscribe to encoder ticks
        self.subscription = self.create_subscription(
            JointState,  
            '/encoder_ticks',  
            self.encoder_callback,
            10
        )

        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publish joint states for wheel joints
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Robot parameters (Adjust based on real robot)
        self.wheel_radius = 0.055  # meters
        self.wheel_base = 0.40  # meters
        self.ticks_per_rev_left = 5700  # Encoder ticks per wheel revolution
        self.ticks_per_rev_right = 5500  # Encoder ticks per wheel revolution

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_ticks = [0, 0]
        self.last_time = self.get_clock().now()

        # Joint angles (for continuous joint movement)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

    def encoder_callback(self, msg: JointState):
        if len(msg.position) < 2:
            self.get_logger().warn("Encoder ticks data missing!")
            return
        
        left_ticks = msg.position[0]
        right_ticks = msg.position[1]

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        # Compute wheel displacement
        left_wheel_distance = (left_ticks - self.prev_ticks[0]) * (2 * math.pi * self.wheel_radius / self.ticks_per_rev_left)
        right_wheel_distance = (right_ticks - self.prev_ticks[1]) * (2 * math.pi * self.wheel_radius / self.ticks_per_rev_right)
        self.prev_ticks = [left_ticks, right_ticks]

        # Compute linear and angular velocity
        v = (right_wheel_distance + left_wheel_distance) / (2 * dt)
        omega = (right_wheel_distance - left_wheel_distance) / (self.wheel_base * dt)

        # Update robot pose
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Update continuous joint angles (simulate wheel rotation)
        self.left_wheel_angle += left_wheel_distance / self.wheel_radius
        self.right_wheel_angle += right_wheel_distance / self.wheel_radius

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

        # Publish TF transform
        odom_tf = TransformStamped()
        odom_tf.header.stamp = self.get_clock().now().to_msg()
        odom_tf.header.frame_id = 'odom'
        odom_tf.child_frame_id = 'base_link'

        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation.z = math.sin(self.theta / 2.0)
        odom_tf.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(odom_tf)

        # Publish joint states for wheel rotation
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["left_wheel_joint", "right_wheel_joint"]
        joint_state_msg.position = [self.left_wheel_angle, self.right_wheel_angle]  # Rotating joints
        self.joint_state_pub.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
