import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import numpy as np
from scipy.optimize import least_squares, differential_evolution
import csv
import os
import datetime
import time
from .magnetic_math import MagneticDipole, cost_bml_math

# === 卡尔曼滤波器 ===
class SimpleKalmanFilter:
    def __init__(self, dt=0.1):
        self.x = np.zeros(6) 
        self.P = np.eye(6) * 1.0
        self.F = np.eye(6)
        self.H = np.zeros((3, 6)); self.H[0,0]=1; self.H[1,1]=1; self.H[2,2]=1      
        self.Q = np.eye(6) * 0.0001 
        self.R = np.eye(3) * (0.001 ** 2) 

    def predict(self, dt):
        self.F[0,3] = dt; self.F[1,4] = dt; self.F[2,5] = dt
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x[:3]

    def update(self, z_meas):
        y = z_meas - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(6) - np.dot(K, self.H), self.P)
        return self.x[:3]


# === 解算节点 (集成全局搜索 + LM局部追踪) ===
class LocalizationSolver(Node):
    def __init__(self):
        super().__init__('localization_solver')
        self.model = MagneticDipole()
        
        self.kf_tml = SimpleKalmanFilter(dt=0.1)
        self.kf_bml = SimpleKalmanFilter(dt=0.1)
        
        self.sensors = []
        for x in [-0.04, 0, 0.04]:
            for y in [-0.04, 0, 0.04]:
                self.sensors.append([x, y, 0.0045])
        self.sensors = np.array(self.sensors)

        self.true_pos = None
        self.last_time = time.time()
        
        self.kf_tml_pos = np.zeros(3)
        self.kf_bml_pos = np.zeros(3)
        
        # 用于判断是否是第一帧数据，以决定是否启动全局搜索
        self.is_bml_init = False
        self.is_tml_init = False
        self.last_m_bml = np.array([0.0, 0.0, 1.0])
        self.last_m_tml = np.array([0.0, 0.0, 1.0])

        self.print_counter = 0
        self.csv_path = os.path.expanduser('~/ros2_ws/final_log.csv')
        self.init_csv()

        self.create_timer(0.1, self.update_and_record) # 10Hz
        self.sub_truth = self.create_subscription(ModelStates, '/gazebo/model_states', self.truth_cb, 10)
        self.sub_data = self.create_subscription(Float64MultiArray, '/sensor_measurements', self.solve_cb, 10)

    def init_csv(self):
        with open(self.csv_path, mode='w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Time', 'True_Pos', 'TML_Final', 'BML_Final', 'TML_Err(mm)', 'BML_Err(mm)'
            ])
        print(f"✅ Log initialized: {self.csv_path}")
        print(f"{'Time':<10} | {'True Z':<8} | {'TML Err':<10} | {'BML Err':<10}")
        print("-" * 60)

    def truth_cb(self, msg):
        try:
            idx = msg.name.index('my_magnet')
            p = msg.pose[idx].position
            self.true_pos = np.array([p.x, p.y, p.z])
        except ValueError: pass

    def solve_cb(self, msg):
        if self.true_pos is None: return
        data = np.array(msg.data).reshape(9, 3)

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 1.0: dt = 0.1
        
        # --- 全局搜索的边界 (x, y, z, mx, my, mz) ---
        search_bounds = [(-0.1, 0.1), (-0.1, 0.1), (0.01, 0.2), (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)]

        # ==================== BML 解算流程 ====================
        try:
            if not self.is_bml_init:
                # 【阶段一：首次定位】使用全局差分进化算法 (完美替代 PSO)
                self.get_logger().info("BML: 正在进行首次全局搜索...")
                
                res_global = differential_evolution(
                    lambda x: np.sum(cost_bml_math(x, data, self.sensors)**2),
                    search_bounds, maxiter=20, popsize=10
                )
                x0_bml = res_global.x
                self.is_bml_init = True
                self.get_logger().info("BML: 全局搜索完成！切入 LM 连续追踪模式。")
            else:
                # 【阶段二：连续追踪】使用卡尔曼滤波器预测的下一帧位置作为 LM 算法的初值
                pred_pos = self.kf_bml.predict(dt)
                x0_bml = np.concatenate((pred_pos, self.last_m_bml))

            res_bml = least_squares(cost_bml_math, x0_bml, args=(data, self.sensors), method='lm', 
                                  ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=50)
            
            if np.linalg.norm(res_bml.x[0:3]) < 0.5:
                self.kf_bml_pos = self.kf_bml.update(res_bml.x[0:3])
                self.last_m_bml = res_bml.x[3:6]
            else:
                self.is_bml_init = False 
                
        except Exception as e: 
            self.get_logger().error(f"BML Solve Error: {e}")
            self.is_bml_init = False

        # ==================== TML 解算流程 ====================
        try:
            if not self.is_tml_init:
                self.get_logger().info("TML: 正在进行首次全局搜索...")
                res_global_tml = differential_evolution(
                    lambda x: np.sum(self.cost_tml(x, data)**2),
                    search_bounds, maxiter=20, popsize=10
                )
                x0_tml = res_global_tml.x
                self.is_tml_init = True
            else:
                pred_pos_tml = self.kf_tml.predict(dt)
                x0_tml = np.concatenate((pred_pos_tml, self.last_m_tml))

            res_tml = least_squares(self.cost_tml, x0_tml, args=(data,), method='lm', 
                                  ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=50)
            
            if np.linalg.norm(res_tml.x[0:3]) < 0.5:
                self.kf_tml_pos = self.kf_tml.update(res_tml.x[0:3])
                self.last_m_tml = res_tml.x[3:6]
            else:
                self.is_tml_init = False
                
        except Exception as e:
            self.get_logger().error(f"TML Solve Error: {e}")
            self.is_tml_init = False

    def update_and_record(self):
        if self.true_pos is None or not self.is_bml_init or not self.is_tml_init: 
            return
        
        err_tml = np.linalg.norm(self.kf_tml_pos - self.true_pos) * 1000
        err_bml = np.linalg.norm(self.kf_bml_pos - self.true_pos) * 1000

        with open(self.csv_path, mode='a', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            t = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-4]
            writer.writerow([
                t, 
                self.fmt(self.true_pos), 
                self.fmt(self.kf_tml_pos), 
                self.fmt(self.kf_bml_pos), 
                f"{err_tml:.4f}", f"{err_bml:.4f}" 
            ])
        
        self.print_counter += 1
        if self.print_counter % 10 == 0:
            t_str = datetime.datetime.now().strftime("%H:%M:%S")
            print(f"{t_str:<10} | {self.true_pos[2]:<8.3f} | {err_tml:<10.4f} | {err_bml:<10.4f}")

    def fmt(self, v): return f"({v[0]:.5f}, {v[1]:.5f}, {v[2]:.5f})"

    # TML 目标函数 (保持在节点内，因为它只做简单的差值)
    def cost_tml(self, x, data):
        pos=x[0:3]; m=x[3:6]
        res=[]
        for i,s in enumerate(self.sensors):
            bc = self.model.calculate_B(s,pos,m); br=data[i]
            res.extend(bc - br)
        return np.array(res)

def main():
    rclpy.init()
    rclpy.spin(LocalizationSolver())
    rclpy.shutdown()