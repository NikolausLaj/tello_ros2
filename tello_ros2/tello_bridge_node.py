
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

        self.velocity = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.last_time = self.get_clock().now()
        self.q, self.ax, self.ay, self.az = 0.0, 0.0, 0.0, 0.0

        self.declare_parameter('imu_rate', 20.0)
        self.declare_parameter('pose_rate', 200.0)
        self.declare_parameter('tf_drone', 'drone')
        self.declare_parameter('tf_base', 'base_link')
        self.declare_parameter('tf_world', 'odom')
        self.declare_parameter('debug', True)

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.tf_drone = self.get_parameter('tf_drone').get_parameter_value().string_value
        self.tf_base = self.get_parameter('tf_base').get_parameter_value().string_value
        self.tf_world = self.get_parameter('tf_world').get_parameter_value().string_value
        imu_rate = self.get_parameter('imu_rate').get_parameter_value().double_value
        pose_rate = self.get_parameter('pose_rate').get_parameter_value().double_value

        self.get_logger().info('TelloBridgeNode has been started.')
        self._tello = djitellopy.Tello()
        self._connectToTello()

        # Timers
        self.imu_request_timer = self.create_timer(1.0/imu_rate, self.attitudeCallback)
        self.pose_request_timer = self.create_timer(1.0/imu_rate, self.poseCallback)

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, 'tello/imu', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'tello/odom', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


    def _connectToTello(self):
        self._tello.connect()
        self.get_logger().info('Connected to Tello drone.')


    def poseCallback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Acceleration (gravity compensation)
        ax = self._tello.get_acceleration_x() / 100.0
        ay = self._tello.get_acceleration_y() / 100.0
        az = self._tello.get_acceleration_z() / 100.0 - 9.81

        # Integrate acceleration to velocity
        self.velocity[0] += ax * dt
        self.velocity[1] += ay * dt
        self.velocity[2] += az * dt

        # Integrate velocity to position
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt

        # Use TOF/barometer for altitude correction if available
        try:
            self.position[2] = self._tello.get_distance_tof() / 100.0
        except Exception:
            pass

    def attitudeCallback(self):
        # Orientation
        roll = -self._tello.get_roll() * math.pi / 180.0
        pitch = self._tello.get_pitch() * math.pi / 180.0
        yaw = self._tello.get_yaw() * math.pi / 180.0
        self.q = euler2quat(roll, pitch, yaw, axes='sxyz')
        print(self.q)
        self.computeIMU()
        self.computeOdometry()
        self.broadcastTF()

    def computeIMU(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tf_drone
        msg.linear_acceleration.x = self.ax
        msg.linear_acceleration.y = self.ay
        msg.linear_acceleration.z = self.az + 9.81  # publish raw accel (add gravity back for IMU msg)

        # Angular velocity not available, set to zero
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.orientation.x = self.q[1]
        msg.orientation.y = self.q[2]
        msg.orientation.z = self.q[3]
        msg.orientation.w = self.q[0]

        self.imu_publisher.publish(msg)

        if self.debug:
            self.get_logger().info('IMU published: Accel[%.2f, %.2f, %.2f], Orient[%.2f, %.2f, %.2f, %.2f]' %
                (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                 msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))


    def computeOdometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.tf_world
        odom_msg.child_frame_id = self.tf_base
        # Use estimated position
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]
        odom_msg.pose.pose.orientation.x = self.q[1]
        odom_msg.pose.pose.orientation.y = self.q[2]
        odom_msg.pose.pose.orientation.z = self.q[3]
        odom_msg.pose.pose.orientation.w = self.q[0]
        # Use estimated velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]

        self.odom_publisher.publish(odom_msg)

        if self.debug:
            self.get_logger().info('Odometry published: Pos[%.2f, %.2f, %.2f], Vel[%.2f, %.2f, %.2f], Orient[%.2f, %.2f, %.2f, %.2f]' %
                (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z,
                 odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z,
                 odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                 odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w))


    def broadcastTF(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.tf_world
        t.child_frame_id = self.tf_base
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        t.transform.rotation.x = self.q[1]
        t.transform.rotation.y = self.q[2]
        t.transform.rotation.z = self.q[3]
        t.transform.rotation.w = self.q[0]
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
