import numpy as np
from scipy.optimize import least_squares
from magnetic_math import MagneticDipole  # 调用

# --- 1. 搭建仿真场景 ---
# 创建一个 3x3 的传感器阵列 (模拟论文里的阵列)
sensors = []
for x in [-0.04, 0, 0.04]:  # 间距 40mm
    for y in [-0.04, 0, 0.04]:
        sensors.append([x, y, 0])
sensors = np.array(sensors)

# 设定一个“真实”的磁铁位置 (假装不知道，看算法能不能算出来)
real_magnet_pos = np.array([0.02, 0.03, 0.15]) # x=2cm, y=3cm, z=15cm
real_magnet_moment = np.array([0, 0, 1])       # 磁矩竖直向上

# --- 2. 造数据 ---
model = MagneticDipole()
measured_data = []

print(f"正在生成 {len(sensors)} 个传感器的仿真数据...")
for sensor_pos in sensors:
    # 算出理论磁场
    b = model.calculate_B(sensor_pos, real_magnet_pos, real_magnet_moment)
    # 加入微小噪声模拟真实环境
    noise = np.random.normal(0, 1e-9, 3) 
    measured_data.append(b + noise)
measured_data = np.array(measured_data)

# --- 3. 算法核心: 误差函数 ---
# 算法通过这个函数知道自己猜得“有多偏”
def error_function(guess_params):
    guess_pos = guess_params[0:3] # 当前猜测的位置
    guess_moment = real_magnet_moment # 假设磁矩方向已知
    
    residuals = []
    for i, sensor_pos in enumerate(sensors):
        # 猜的磁场
        b_calc = model.calculate_B(sensor_pos, guess_pos, guess_moment)
        # 实际测的
        b_real = measured_data[i]
        # 记录差距
        residuals.extend(b_calc - b_real)
    return np.array(residuals)

# --- 4. 运行优化算法 (TML) ---
print("开始反向解算位置...")
initial_guess = np.array([0.0, 0.0, 0.1]) # 随便给个初始猜测
result = least_squares(error_function, initial_guess, method='lm') # LM算法

# --- 5. 结果对比 ---
estimated_pos = result.x
error_dist = np.linalg.norm(estimated_pos - real_magnet_pos)

print("\n" + "=" * 30)
print(f"真实位置: {real_magnet_pos}")
print(f"解算位置: {estimated_pos}")
print(f"误差距离: {error_dist * 1000:.4f} mm")
print("=" * 30)

if error_dist < 1.0:
    print("✅ 成功！TML算法复现完成！")
else:
    print("❌ 误差过大，请检查代码。")