
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import djitellopy
import math
from transforms3d.euler import euler2quat
import tf2_ros

class TelloBridgeNode(Node):
    def __init__(self):
        super().__init__('tello_bridge_node')
        self.declare_parameter('imu_rate', 10.0)
        self.declare_parameter('tf_drone', 'drone')
        self.declare_parameter('tf_base', 'base_link')
        self.declare_parameter('tf_world', 'odom')
        self.declare_parameter('debug', True)

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.tf_drone = self.get_parameter('tf_drone').get_parameter_value().string_value
        self.tf_base = self.get_parameter('tf_base').get_parameter_value().string_value
        self.tf_world = self.get_parameter('tf_world').get_parameter_value().string_value
        imu_rate = self.get_parameter('imu_rate').get_parameter_value().double_value

        self.get_logger().info('TelloBridgeNode has been started.')
        self._tello = djitellopy.Tello()
        self._connectToTello()

        self.imu_request_timer = self.create_timer(1.0/imu_rate, self.attitudeCallback)
        self.imu_publisher = self.create_publisher(Imu, 'tello/imu', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'tello/odom', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


    def _connectToTello(self):
        self._tello.connect()
        self.get_logger().info('Connected to Tello drone.')


    def attitudeCallback(self):
        roll = self._tello.get_roll() * math.pi / 180.0
        pitch = self._tello.get_pitch() * math.pi / 180.0
        yaw = self._tello.get_yaw() * math.pi / 180.0

        q_wxyz = euler2quat(roll, pitch, yaw, axes='sxyz')
        self.computeIMU(q_wxyz)
        self.computeOdometry(q_wxyz)
        self.broadcastTF(q_wxyz)


    def computeIMU(self, q_wxyz):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tf_drone
        msg.linear_acceleration.x = self._tello.get_acceleration_x() / 100.0
        msg.linear_acceleration.y = self._tello.get_acceleration_y() / 100.0
        msg.linear_acceleration.z = self._tello.get_acceleration_z() / 100.0
        # Angular velocity not available, set to zero
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.orientation.x = q_wxyz[1]
        msg.orientation.y = q_wxyz[2]
        msg.orientation.z = q_wxyz[3]
        msg.orientation.w = q_wxyz[0]

        self.imu_publisher.publish(msg)

        if self.debug:
            self.get_logger().info('IMU published: Accel[%.2f, %.2f, %.2f], Orient[%.2f, %.2f, %.2f, %.2f]' %
                (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                 msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))


    def computeOdometry(self, q_wxyz):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.tf_world
        odom_msg.child_frame_id = self.tf_base
        # Position not available, set to zero
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q_wxyz[1]
        odom_msg.pose.pose.orientation.y = q_wxyz[2]
        odom_msg.pose.pose.orientation.z = q_wxyz[3]
        odom_msg.pose.pose.orientation.w = q_wxyz[0]
        odom_msg.twist.twist.linear.x = float(self._tello.get_speed_x()) / 100.0
        odom_msg.twist.twist.linear.y = float(self._tello.get_speed_y()) / 100.0
        odom_msg.twist.twist.linear.z = float(self._tello.get_speed_z()) / 100.0

        self.odom_publisher.publish(odom_msg)

        if self.debug:
            self.get_logger().info('Odometry published: Vel[%.2f, %.2f, %.2f], Orient[%.2f, %.2f, %.2f, %.2f]' %
                (odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z,
                 odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                 odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w))


    def broadcastTF(self, q_wxyz):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.tf_world
        t.child_frame_id = self.tf_base
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q_wxyz[1]
        t.transform.rotation.y = q_wxyz[2]
        t.transform.rotation.z = q_wxyz[3]
        t.transform.rotation.w = q_wxyz[0]
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TelloBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
