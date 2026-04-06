import numpy as np

class MagneticDipole:
    def __init__(self, mu_r=1.0, M0=1.0, volume=1.0):
        self.mu0 = 4 * np.pi * 1e-7
        # 必须算出真实的 BT 物理量级
        self.BT = (self.mu0 * mu_r * M0 * volume) / (4 * np.pi)

    def calculate_B(self, sensor_pos, magnet_pos, magnet_moment_vec):
        P = np.array(sensor_pos)
        T = np.array(magnet_pos)
        H = np.array(magnet_moment_vec)

        r_vec = P - T 
        r_val = np.linalg.norm(r_vec)
        if r_val < 1e-9: 
            return np.zeros(3)

        r_hat = r_vec / r_val
        dot_val = np.dot(H, r_hat)
        term1 = 3 * dot_val * r_hat
        term2 = H
        
        # 乘上 BT
        B = self.BT * (term1 - term2) / (r_val ** 3)
        return B

def cost_bml_math(x, data, sensors):

    pos = x[0:3]
    m = x[3:6]
    
    # 确保磁矩向量是单位向量
    m_norm = np.linalg.norm(m) + 1e-12
    m_unit = m / m_norm
    
    res = []
    
    # --- 步骤 1：构建坐标系旋转矩阵 R_go ---
    Z_axis = np.array([0.0, 0.0, 1.0])
    cross_v = np.cross(Z_axis, m_unit)
    cos_theta = np.dot(Z_axis, m_unit)
    
    if cos_theta < -0.999999: 
        R_go = -np.eye(3)
        R_go[2, 2] = 1.0
    else:
        s_mat = np.array([
            [0, -cross_v[2], cross_v[1]], 
            [cross_v[2], 0, -cross_v[0]], 
            [-cross_v[1], cross_v[0], 0]
        ])
        R_go = np.eye(3) + s_mat + np.dot(s_mat, s_mat) * (1.0 / (1.0 + cos_theta))
        
    R_og = R_go.T 

    for i, s in enumerate(sensors):
        br = data[i] 
        
        # --- 步骤 2：测量磁场转换到观察坐标系 ---
        B_o = np.dot(R_og, br)
        Bx, By, Bz = B_o
        B_xy = np.sqrt(Bx**2 + By**2)
        
        # --- 步骤 3：核心映射函数 g(θ') ---
        if B_xy < 1e-9:
            v_o = np.array([0.0, 0.0, np.sign(Bz)])
        else:
            tan_tp = B_xy / Bz if abs(Bz) > 1e-9 else 1e9 * np.sign(Bz)
            t = (-3.0 + np.sqrt(9.0 + 8.0 * tan_tp**2)) / (2.0 * tan_tp)
            
            cos_phi = Bx / B_xy
            sin_phi = By / B_xy
            
            v_o = np.array([t * cos_phi, t * sin_phi, 1.0])
            v_o = v_o / np.linalg.norm(v_o)
            
            if Bz < 0:
                v_o[2] = -v_o[2]
                v_o = v_o / np.linalg.norm(v_o)
                
        # --- 步骤 4：转回全局坐标系 ---
        v_g = np.dot(R_go, v_o)
        
        # --- 步骤 5：构建叉乘目标函数 ---
        geo_vec = pos - s
        geo_vec_norm = np.linalg.norm(geo_vec) + 1e-12
        geo_vec_unit = geo_vec / geo_vec_norm
        
        cross_err = np.cross(geo_vec_unit, v_g)
        res.extend(cross_err)

    return np.array(res)