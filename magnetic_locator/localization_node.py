import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import numpy as np
from scipy.optimize import least_squares
from .magnetic_math import MagneticDipole # 确保上一段代码已保存

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # 1. 传感器配置 (3x3 阵列)
        self.sensor_positions = []
        for x in [-0.04, 0, 0.04]:
            for y in [-0.04, 0, 0.04]:
                self.sensor_positions.append([x, y, 0.0045]) # 0.0045是之前算的传感器中心高度
        self.sensor_positions = np.array(self.sensor_positions)
        
        self.model = MagneticDipole()
        self.magnet_moment = np.array([0, 0, 1.0]) # 假设磁矩强度和方向已知
        
        # 2. 订阅 Gazebo 状态，获取磁铁“真值”并触发解算
        self.subscription = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.listener_callback, 10)
        
        self.get_logger().info('定位解算节点已启动，正在监听磁场数据...')

    def listener_callback(self, msg):
        # 找到磁铁在列表中的索引
        try:
            idx = msg.name.index('my_magnet')
        except ValueError:
            return

        # 拿到磁铁的 Gazebo 真值位置
        true_pos = np.array([
            msg.pose[idx].position.x,
            msg.pose[idx].position.y,
            msg.pose[idx].position.z
        ])

        # --- 第一步：模拟传感器读取数据 (注入少量噪声模拟真实情况) ---
        measured_B = []
        for p_s in self.sensor_positions:
            b = self.model.calculate_B(p_s, true_pos, self.magnet_moment)
            noise = np.random.normal(0, 1e-10, 3) # 注入 0.1nT 级别的噪声
            measured_B.append(b + noise)
        measured_B = np.array(measured_B)

        # --- 第二步：运行 TML 算法 ---
        est_pos_tml = self.run_tml(measured_B)
        
        # --- 第三步：运行 BML 算法 ---
        est_pos_bml = self.run_bml(measured_B)

        # --- 第四步：对比误差 ---
        err_tml = np.linalg.norm(est_pos_tml - true_pos) * 1000 # mm
        err_bml = np.linalg.norm(est_pos_bml - true_pos) * 1000 # mm

        self.get_logger().info(f'\n真值位置: {true_pos}\n'
                               f'TML 误差: {err_tml:.2f} mm | BML 误差: {err_bml:.2f} mm')

    def run_tml(self, measured_B):
        def tml_residuals(guess_pos):
            res = []
            for i, p_s in enumerate(self.sensor_positions):
                b_calc = self.model.calculate_B(p_s, guess_pos, self.magnet_moment)
                res.extend(b_calc - measured_B[i])
            return np.array(res)
        
        # 初始猜测给一个略微偏离的值
        start_guess = np.array([0.0, 0.0, 0.1])
        res = least_squares(tml_residuals, start_guess, method='lm')
        return res.x

    def run_bml(self, measured_B):
        def bml_residuals(guess_pos):
            res = []
            for i, p_s in enumerate(self.sensor_positions):
                b_calc = self.model.calculate_B(p_s, guess_pos, self.magnet_moment)
                b_real = measured_B[i]
                
                # BML 核心：方向归一化后的叉积
                v_calc = b_calc / (np.linalg.norm(b_calc) + 1e-12)
                v_real = b_real / (np.linalg.norm(b_real) + 1e-12)
                
                res.extend(np.cross(v_calc, v_real))
            return np.array(res)
        
        start_guess = np.array([0.0, 0.0, 0.1])
        res = least_squares(bml_residuals, start_guess, method='lm')
        return res.x

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()