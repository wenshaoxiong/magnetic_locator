import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np

CSV_PATH = os.path.expanduser('~/ros2_ws/final_log.csv')

def parse_col(df, name):
    if name not in df.columns: return None, None, None
    s = df[name].astype(str).str.replace('(', '').str.replace(')', '')
    d = s.str.split(', ', expand=True).astype(float)
    return d[0].to_numpy(), d[1].to_numpy(), d[2].to_numpy()

def plot_results():
    if not os.path.exists(CSV_PATH):
        print(f"❌ File not found: {CSV_PATH}")
        return

    print(f"✅ Reading data: {CSV_PATH}")
    try: df = pd.read_csv(CSV_PATH, encoding='utf-8-sig')
    except: df = pd.read_csv(CSV_PATH, encoding='gbk')

    # 解析全部数据用于画图（不剔除任何点）
    tx, ty, tz = parse_col(df, 'True_Pos')
    tmx, tmy, tmz = parse_col(df, 'TML_Final')
    bx, by, bz = parse_col(df, 'BML_Final')

    # 【逻辑修改】仅在计算平均值时，剔除前 800 步的瞬间飞点
    CUT_STEP = 800 
    if len(df) > CUT_STEP:
        # 使用 .iloc[CUT_STEP:] 截取平稳段数据计算平均值
        e_tml = np.mean(df['TML_Err(mm)'].iloc[CUT_STEP:])
        e_bml = np.mean(df['BML_Err(mm)'].iloc[CUT_STEP:])
        print(f"✂️  画图保留全部数据。已剔除前 {CUT_STEP} 步不稳定数据，使用剩余 {len(df)-CUT_STEP} 步计算平均误差。")
    else:
        e_tml = np.mean(df['TML_Err(mm)'])
        e_bml = np.mean(df['BML_Err(mm)'])

    fig = plt.figure(figsize=(16, 8))

    # --- 左图：3D 轨迹 ---
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.plot(tx, ty, tz, 'k-', linewidth=3, alpha=0.3, label='Ground Truth')
    ax1.plot(tmx, tmy, tmz, 'b-', linewidth=1.5, alpha=0.7, label='TML+KF')
    ax1.plot(bx, by, bz, 'r-', linewidth=1.5, alpha=0.7, label='BML+KF')
    
    ax1.set_title(f'Trajectory (Noise=2%)')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
    ax1.legend()

    # --- 右图：误差分析 ---
    ax2 = fig.add_subplot(1, 2, 2)
    
    # 【Bug修复】使用 np.arange 创建标准的 X 轴，并将 DataFrame 的列强制转为 numpy 数组
    steps = np.arange(len(df))
    err_tml_array = df['TML_Err(mm)'].to_numpy()
    err_bml_array = df['BML_Err(mm)'].to_numpy()

    # 画图使用全部数据，图例标签显示平稳段的平均误差
    ax2.plot(steps, err_tml_array, 'b-', linewidth=1, alpha=0.6, label=f'TML Mean (Stable): {e_tml:.4f}mm')
    ax2.plot(steps, err_bml_array, 'r-', linewidth=1, alpha=0.8, label=f'BML Mean (Stable): {e_bml:.4f}mm')
    
    ax2.set_title('Error Analysis')
    ax2.set_xlabel('Time Step (0.1s)')
    ax2.set_ylabel('Error (mm)')
    ax2.legend(fontsize=12)
    ax2.grid(True, linestyle='--')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_results()