import numpy as np

class KalmanFilter:
    def __init__(self, dt=0.05): # dt 是采样时间间隔，假设约 20Hz
        # 1. 状态向量 X = [x, y, z, vx, vy, vz] (位置 + 速度)
        self.x = np.zeros(6) 
        
        # 2. 状态转移矩阵 F (物理模型：位置 = 上次位置 + 速度*时间)
        self.F = np.eye(6)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt
        
        # 3. 测量矩阵 H (我们只能测量位置 x,y,z，测不到速度)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1
        
        # 4. 协方差矩阵 P (初始不确定性，设大一点)
        self.P = np.eye(6) * 1.0
        
        # 5. 过程噪声 Q (代表物体运动的不确定性，比如突然变速)
        self.Q = np.eye(6) * 0.001
        
        # 6. 测量噪声 R (代表 BML 算法解算的不准程度)
        # 之前看到 BML 误差约 4mm (0.004m)，这里平方一下
        self.R = np.eye(3) * (0.004 ** 2)

    def predict(self):
        # 预测下一步状态: X_pred = F * X
        self.x = np.dot(self.F, self.x)
        # 更新协方差: P_pred = F * P * F.T + Q
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x[:3] # 返回预测的位置

    def update(self, z_meas):
        # z_meas 是 BML 刚刚算出来的 [x, y, z]
        
        # 计算卡尔曼增益 K
        # K = P * H.T * inv(H * P * H.T + R)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # 更新状态 X = X + K * (z - H * X)
        y = z_meas - np.dot(self.H, self.x) # 测量残差
        self.x = self.x + np.dot(K, y)
        
        # 更新协方差 P = (I - K * H) * P
        I = np.eye(6)
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        
        return self.x[:3] # 返回最优估计的位置