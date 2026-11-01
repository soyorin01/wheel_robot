import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def quat(roll, pitch, yaw):
    cr, sr = math.cos(math.radians(roll) / 2), math.sin(math.radians(roll) / 2)
    cp, sp = math.cos(math.radians(pitch) / 2), math.sin(math.radians(pitch) / 2)
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)

class Fake(Node):
    def __init__(self):
        super().__init__("fake_sensors")
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
        self.t0 = time.monotonic()
        self.create_timer(0.02, self.tick)
    def tick(self):
        t = time.monotonic() - self.t0
        phase = t % 6.5
        roll = 6.0 if 3.5 <= phase < 4.7 else 0.0
        yaw = 0.0 if t < 7.5 else -min(120.0 * (t - 7.5), 130.0)
        speed = 0.50 if abs(yaw) < 90.0 else 0.0
        qw, qx, qy, qz = quat(roll, 0.0, yaw)
        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        # legacy 顺序：q0->x, q1->y, q2->z, q3->w，即真 w 放 x
        imu.orientation.x, imu.orientation.y = qw, qx
        imu.orientation.z, imu.orientation.w = qy, qz
        self.imu_pub.publish(imu)
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.twist.twist.linear.x = speed
        self.odom_pub.publish(od)

rclpy.init()
n = Fake()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
