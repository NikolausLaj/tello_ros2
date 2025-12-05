import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Empty
from sensor_msgs.msg import Imu, Range, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import djitellopy
import time
import math
from transforms3d.euler import euler2quat
import tf2_ros
import statistics
from cv_bridge import CvBridge

# from tello_ros2.msg import TelloStatus

# --------------------------------------------------------------------------------------------------

class TelloBridgeNode(Node):
    def __init__(self):
        super().__init__('tello_bridge_node')

        self._position = [0.0, 0.0, 0.0]
        self._last_time = self.get_clock().now()
        self._home_altitude = 0.0
        self._old_abs_alt = 0.0
        self._abs_alt_median = []
        self._q = 0.0

        self.declare_parameter('imu_rate', 20.0)
        self.declare_parameter('pose_rate', 20.0)
        # self.declare_parameter('status_rate', 1.0)
        self.declare_parameter('image_rate', 30.0)
        self.declare_parameter('tf_drone', 'drone')
        self.declare_parameter('tf_base', 'base_link')
        self.declare_parameter('tf_odom', 'odom')
        self.declare_parameter('tf_map', 'map')
        self.declare_parameter('debug', False)
        self.declare_parameter('alpha', 0.925)
        self.declare_parameter('buffer_length', 5)
        self.declare_parameter('average_time', 5.0)
        self.declare_parameter('average_rate', 20.0)
        self.declare_parameter('filter_barometer', False)

        imu_rate = self.get_parameter('imu_rate').get_parameter_value().double_value
        pose_rate = self.get_parameter('pose_rate').get_parameter_value().double_value
        # status_rate = self.get_parameter('status_rate').get_parameter_value().double_value
        image_rate = self.get_parameter('image_rate').get_parameter_value().double_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.tf_drone = self.get_parameter('tf_drone').get_parameter_value().string_value
        self.tf_base = self.get_parameter('tf_base').get_parameter_value().string_value
        self.tf_odom = self.get_parameter('tf_odom').get_parameter_value().string_value
        self.tf_map = self.get_parameter('tf_map').get_parameter_value().string_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.buffer_length = self.get_parameter('buffer_length').get_parameter_value().integer_value
        self.average_time = self.get_parameter('average_time').get_parameter_value().double_value
        self.average_rate = self.get_parameter('average_rate').get_parameter_value().double_value
        self.filter_barometer = self.get_parameter('filter_barometer').get_parameter_value().bool_value

        self.get_logger().info('TelloBridgeNode has been started.')
        self.tello = djitellopy.Tello()
        self.cv_bridge = CvBridge()

        self._connectToTello()
        self._averageHomeAltitude()
        self.tello.streamon()


        # Timers
        self.imu_request_timer = self.create_timer(1.0/imu_rate, self.attitudeCallback)
        self.pose_request_timer = self.create_timer(1.0/pose_rate, self.altitudeCallback)
        # self.status_request_timer = self.create_timer(1.0/status_rate, self.statusCallback)
        self.image_request_timer = self.create_timer(1.0/image_rate, self.imageCallback)

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, 'tello/imu', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'tello/odom', 10)
        self.rel_alt_publisher = self.create_publisher(Range, 'tello/relative_altitude', 10)
        # self.status_publisher = self.create_publisher(TelloStatus, 'tello/status', 10)
        self.image_publisher = self.create_publisher(Image, 'tello/image_raw', 10)

        # Subscribers
        self.create_subscription(Empty, 'takeoff', self.subTakeoff, 1)
        self.create_subscription(Empty, 'land', self.subLand, 1)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Broadcast static map->odom transform (identity)
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = self.tf_map
        static_t.child_frame_id = self.tf_odom

        static_t.transform.translation.x = 0.0
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.0

        static_t.transform.rotation.x = 0.0
        static_t.transform.rotation.y = 0.0
        static_t.transform.rotation.z = 0.0
        static_t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([static_t])

        self.get_logger().info('TelloBridge Node Running...')

# --------------------------------------------------------------------------------------------------

    def _averageHomeAltitude(self):
        self.get_logger().info(f'Calculating average home altitude for {self.average_time} seconds at rate {self.average_rate} Hz...')
        start_time = self.get_clock().now()
        barometer_measurements = []
        try:
            while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < self.average_time:
                barometer_measurements.append(self.tello.get_barometer())
                time.sleep(1/self.average_rate)
            
            self._home_altitude = (sum(barometer_measurements) / len(barometer_measurements)) / 100.0
            self._old_abs_alt = self._home_altitude
            self.get_logger().info(f'{len(barometer_measurements)} measurements taken over {self.average_time} seconds.')
            self.get_logger().info(f'Average home altitude: {self._home_altitude:.2f} m')
        except Exception as e:
            self.get_logger().error(f'Error getting home altitude: {e}')
            return 0.0

# --------------------------------------------------------------------------------------------------

    def _connectToTello(self):
        self.tello.connect()
        self.get_logger().info('Connected to Tello drone.')

# --------------------------------------------------------------------------------------------------

    def altitudeCallback(self):
        try:
            if self.filter_barometer:
                new_abs_alt = self.tello.get_barometer() / 100.0
                abs_alt_filtered = self.alpha * self._old_abs_alt + ( 1 - self.alpha) * new_abs_alt
                self._abs_alt_median.append(abs_alt_filtered)

                if len(self._abs_alt_median) > self.buffer_length:
                    self._abs_alt_median.pop(0)
                
                self._position[2] = statistics.median(self._abs_alt_median) - self._home_altitude
                self._old_abs_alt = abs_alt_filtered

            else:
                new_abs_alt = self.tello.get_barometer() / 100.0
                self._position[2] = new_abs_alt - self._home_altitude
                
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = self.tf_drone
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.47
            range_msg.min_range = 0.1
            range_msg.max_range = 3.0
            range_msg.range = self.tello.get_distance_tof() / 100.0
            self.rel_alt_publisher.publish(range_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in poseCallback: {e}')

# --------------------------------------------------------------------------------------------------

    def attitudeCallback(self):
        roll = -self.tello.get_roll() * math.pi / 180.0
        pitch = self.tello.get_pitch() * math.pi / 180.0
        yaw = self.tello.get_yaw() * math.pi / 180.0

        self._q = euler2quat(roll, pitch, yaw, axes='sxyz')
        self.computeIMU()
        self.computeOdometry()
        self.broadcastTF()

# --------------------------------------------------------------------------------------------------

    def computeIMU(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tf_drone

        msg.linear_acceleration.x = self.tello.get_acceleration_x()
        msg.linear_acceleration.y = self.tello.get_acceleration_y()
        msg.linear_acceleration.z = self.tello.get_acceleration_z()

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        msg.orientation.x = self._q[1]
        msg.orientation.y = self._q[2]
        msg.orientation.z = self._q[3]
        msg.orientation.w = self._q[0]

        self.imu_publisher.publish(msg)

        if self.debug:
            self.get_logger().info('IMU published: Accel[%.2f, %.2f, %.2f], Orient[%.2f, %.2f, %.2f, %.2f]' %
                (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                 msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
            
# --------------------------------------------------------------------------------------------------

    def imageCallback(self):
        frame = self.tello.get_frame_read()
        msg = self.cv_bridge.cv2_to_imgmsg(frame.frame, encoding="rgb8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tf_drone
        self.image_publisher.publish(msg)

# --------------------------------------------------------------------------------------------------

    # def statusCallback(self):
    #     if self.status_publisher.get_subscription_count > 0:
    #         status_msg = TelloStatus()
    #         status_msg.header.stamp = self.get_clock().now().to_msg()
    #         status_msg.battery_percentage = self._tello.get_battery()
    #         status_msg.flight_time = self._tello.get_flight_time()
    #         status_msg.high_temperature = self._tello.get_high_temperature()
    #         status_msg.low_temperature = self._tello.get_low_temperature()
    #         status_msg.temperature = self._tello.get_temperature()
    #         status_msg.wifi_signal_strength = self._tello.get_wifi_signal_strength()
    #         self.status_publisher.publish(status_msg)

# --------------------------------------------------------------------------------------------------

    def subTakeoff(self, msg):
        self.get_logger().info('Takeoff command received via ROS topic.')
        self.tello.takeoff()

# --------------------------------------------------------------------------------------------------

    def subLand(self, msg):
        self.get_logger().info('Land command received via ROS topic.')
        self.tello.land()

# --------------------------------------------------------------------------------------------------

    def computeOdometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.tf_odom
        odom_msg.child_frame_id = self.tf_base

        odom_msg.pose.pose.position.x = self._position[0]
        odom_msg.pose.pose.position.y = self._position[1]
        odom_msg.pose.pose.position.z = self._position[2]

        odom_msg.pose.pose.orientation.x = self._q[1]
        odom_msg.pose.pose.orientation.y = self._q[2]
        odom_msg.pose.pose.orientation.z = self._q[3]
        odom_msg.pose.pose.orientation.w = self._q[0]

        odom_msg.twist.twist.linear.x = float(self.tello.get_speed_x())
        odom_msg.twist.twist.linear.y = float(self.tello.get_speed_y())
        odom_msg.twist.twist.linear.z = float(self.tello.get_speed_z())

        self.odom_publisher.publish(odom_msg)

        if self.debug:
            self.get_logger().info('Odometry published: Pos[%.2f, %.2f, %.2f], Vel[%.2f, %.2f, %.2f], Orient[%.2f, %.2f, %.2f, %.2f]' %
                (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z,
                 odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z,
                 odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                 odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w))

# --------------------------------------------------------------------------------------------------

    def broadcastTF(self):
        # Broadcast odom -> base_link (drone pose in odom)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.tf_odom
        t.child_frame_id = self.tf_base 

        t.transform.translation.x = self._position[0]
        t.transform.translation.y = self._position[1]
        t.transform.translation.z = self._position[2]

        t.transform.rotation.x = self._q[1]
        t.transform.rotation.y = self._q[2]
        t.transform.rotation.z = self._q[3]
        t.transform.rotation.w = self._q[0]
        self.tf_broadcaster.sendTransform(t)

# --------------------------------------------------------------------------------------------------

    def testFlight(self):
        self.get_logger().info('Starting test flight: takeoff, hover 20s, land.')
        self.tello.takeoff()
        # Schedule landing after 20 seconds using a ROS timer
        self.flight_timer = self.create_timer(20.0, self._land_and_log)

    def _land_and_log(self):
        self.flight_timer.cancel()
        self.tello.land()
        self.get_logger().info('Test flight completed.')

def main():
    rclpy.init()
    node = TelloBridgeNode()
    try:
        # node.testFlight()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
# --------------------------------------------------------------------------------------------------

if __name__ == '__main__':
    main()
