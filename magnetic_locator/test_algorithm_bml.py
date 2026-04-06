import numpy as np
from scipy.optimize import least_squares
from magnetic_math import MagneticDipole

# --- 1. 场景设置 ---
sensors = []
for x in [-0.04, 0, 0.04]:
    for y in [-0.04, 0, 0.04]:
        sensors.append([x, y, 0])
sensors = np.array(sensors)

real_magnet_pos = np.array([0.02, 0.03, 0.15]) 
real_magnet_moment = np.array([0, 0, 1])

# --- 2. 造数据 ---
model = MagneticDipole()
measured_data = []

print(f"正在生成仿真数据...")
for sensor_pos in sensors:
    b = model.calculate_B(sensor_pos, real_magnet_pos, real_magnet_moment)
    # 依然加入噪声，看看BML抗干扰能力如何
    noise = np.random.normal(0, 1e-9, 3) 
    measured_data.append(b + noise)
measured_data = np.array(measured_data)


def error_function_BML(guess_params):
    guess_pos = guess_params[0:3]
    guess_moment = real_magnet_moment 
    
    residuals = []
    
    for i, sensor_pos in enumerate(sensors):
        # 1. 拿到“实际测量”的磁场向量
        b_real = measured_data[i]
        
        # 2. 拿到“当前猜测”的磁场向量
        b_calc = model.calculate_B(sensor_pos, guess_pos, guess_moment)
        
        # --- BML 关键步骤 A: 归一化 (丢弃强度信息，只留方向) ---
        # 论文核心思想：强度 B_T 受环境影响大，不靠谱；但方向比较稳。
        # 变成单位向量： v = v / |v|
        
        # 防止除以0报错，加一个小量 1e-12
        vec_real_dir = b_real / (np.linalg.norm(b_real) + 1e-12)
        vec_calc_dir = b_calc / (np.linalg.norm(b_calc) + 1e-12)
        
        # --- BML 关键步骤 B: 叉积 (Cross Product) ---
        # 物理含义：如果两个向量方向一致，它们的叉积应该为 0。
        # 论文公式 (2.50) 的核心也就是利用叉积来衡量“方向偏了多少”
        
        cross_error = np.cross(vec_calc_dir, vec_real_dir)
        
        # 将叉积的 x,y,z 三个分量都作为误差
        residuals.extend(cross_error)
        
    return np.array(residuals)

# --- 3. 运行解算 ---
print("开始 BML (基于方位) 反向解算...")

# 【关键实验】我们故意给一个非常离谱的初始猜测
# TML 如果用这个初始值 [0.5, 0.5, 0.5] 可能会算飞
# 我们看看 BML 能不能拉回来
initial_guess = np.array([0.01, 0.02, 0.1]) 

print(f"初始猜测位置 : {initial_guess}")

# 调用 LM 算法
result = least_squares(error_function_BML, initial_guess, method='lm')

# --- 4. 结果展示 ---
estimated_pos = result.x
error_dist = np.linalg.norm(estimated_pos - real_magnet_pos)

print("\n" + "=" * 40)
print(f"真实位置: {real_magnet_pos}")
print(f"解算位置: {estimated_pos}")
print(f"误差距离: {error_dist * 1000:.4f} mm")
print("=" * 40)

if error_dist < 5.0:
    print("✅ BML 成功！")
else:
    print("❌ 失败，陷入局部最优。")