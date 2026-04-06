import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from magnetic_interfaces.msg import SensorArrayData
import numpy as np
from scipy.spatial.transform import Rotation as R
from .magnetic_math import MagneticDipole

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        self.model = MagneticDipole()
        self.sensors = []
        for x in [-0.04, 0, 0.04]:
            for y in [-0.04, 0, 0.04]:
                self.sensors.append([x, y, 0.0045])

        self.pub = self.create_publisher(SensorArrayData, '/sensor_measurements', 10)
        self.sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_cb, 10)

    def gazebo_cb(self, msg):
        try:
            idx = msg.name.index('my_magnet')
            pos = msg.pose[idx].position
            ori = msg.pose[idx].orientation

            # 磁铁真实位置
            true_pos = np.array([pos.x, pos.y, pos.z])

            # 获取磁铁实时的真实倾斜姿态 H
            r = R.from_quat([ori.x, ori.y, ori.z, ori.w])
            real_moment = r.apply([0.0, 0.0, 1.0])

            # 计算 IMU 数据 (Roll, Pitch, Yaw)
            euler = r.as_euler('xyz', degrees=False)
            roll, pitch, yaw = euler

            # 采集磁场数据
            measurements = []
            for s_pos in self.sensors:
                b_ideal = self.model.calculate_B(s_pos, true_pos, real_moment)
                # 2% 高斯噪声
                noise = np.random.normal(0, np.linalg.norm(b_ideal) * 0.02, 3)
                b_final = b_ideal + noise
                measurements.extend(b_final.tolist())

            # 构建自定义消息
            out_msg = SensorArrayData()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = 'sensor_array'

            # True Pose (Ground Truth)
            out_msg.true_pose.position.x = pos.x
            out_msg.true_pose.position.y = pos.y
            out_msg.true_pose.position.z = pos.z
            out_msg.true_pose.orientation.x = ori.x
            out_msg.true_pose.orientation.y = ori.y
            out_msg.true_pose.orientation.z = ori.z
            out_msg.true_pose.orientation.w = ori.w

            # IMU RPY
            out_msg.imu_rpy.x = roll
            out_msg.imu_rpy.y = pitch
            out_msg.imu_rpy.z = yaw

            # 磁场数据 (展平的一维数组)
            out_msg.magnetic_fields = measurements

            self.pub.publish(out_msg)

        except ValueError:
            pass

def main():
    rclpy.init()
    rclpy.spin(SensorSimulator())
    rclpy.shutdown()