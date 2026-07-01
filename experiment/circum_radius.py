import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from aruco_opencv_msgs.msg import ArucoDetection
import tf2_ros

# ─── New Circumnavigation Parameters ─────────────────────────────────────────
CENTER_X = 0.0     # Target center X in ground_origin
CENTER_Y = 0.0     # Target center Y in ground_origin
RADIUS   = 0.5     # Target radius in meters
SPEED    = 0.1     # Tangential speed (m/s)
K_RADIAL = 2.0     # Gain to correct distance from center
# ─────────────────────────────────────────────────────────────────────────────

class Circumnavigator(Node):
    def __init__(self):
        super().__init__('circumnavigator')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        try:
            # Get current robot position
            tf = self.tf_buffer.lookup_transform('ground_origin', 'marker_0', rclpy.time.Time())
            x, y = tf.transform.translation.x, tf.transform.translation.y
            
            # 1. Calculate polar coordinates relative to center
            dx = x - CENTER_X
            dy = y - CENTER_Y
            dist_to_center = math.sqrt(dx**2 + dy**2)
            angle_to_center = math.atan2(dy, dx)
            
            # 2. Radial Control: Correct distance to RADIUS
            radial_error = RADIUS - dist_to_center
            linear_x = K_RADIAL * radial_error
            
            # 3. Tangential Control: Orbiting
            # Desired heading is 90 degrees offset from angle_to_center
            # We want to move perpendicular to the radius
            desired_angle = angle_to_center + (math.pi / 2.0)
            
            # Extract current orientation from TF quaternion (simplified for 2D)
            q = tf.transform.rotation
            _, _, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            # Angular velocity to keep tangential path
            angular_z = 2.0 * (desired_angle - yaw) 
            
            # 4. Command
            self._publish_cmd(SPEED, angular_z)
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def _publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def euler_from_quaternion(self, x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return 0, 0, math.atan2(t3, t4)

def main(args=None):
    rclpy.init(args=args)
    node = Circumnavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()