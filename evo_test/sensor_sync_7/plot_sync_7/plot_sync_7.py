import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from matplotlib import rcParams

# 设置字体属性
config = {
    "font.family": 'serif',  # 设置字体类型
    "font.serif": ['Times New Roman'],  # 设置字体为新罗马
    "mathtext.fontset": 'stix',
    "font.size": 12,  # 设置字体大小
    "figure.dpi": 100  # 提高分辨率
}
rcParams.update(config)

def read_tum_data(file_path):
    columns = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
    df = pd.read_csv(file_path, delim_whitespace=True, header=None, names=columns)
    return df

def compute_velocity(df):
    df['dt'] = df['timestamp'].diff()
    df['vx'] = df['tx'].diff() / df['dt']
    df['vy'] = df['ty'].diff() / df['dt']
    df['vz'] = df['tz'].diff() / df['dt']
    return df.dropna()

def compute_rpy(df):
    if all(col in df.columns for col in ['qx', 'qy', 'qz', 'qw']):
        # 过滤掉零范数四元数
        quat_norm = np.sqrt(df[['qx', 'qy', 'qz', 'qw']].pow(2).sum(axis=1))
        non_zero_quat = quat_norm != 0
        df = df[non_zero_quat]
        rotations = R.from_quat(df[['qx', 'qy', 'qz', 'qw']].values)
        df['roll'], df['pitch'], df['yaw'] = rotations.as_euler('xyz', degrees=True).T
    else:
        df['roll'], df['pitch'], df['yaw'] = np.nan, np.nan, np.nan
    return df

def plot_trajectory(dfs, labels):
    plt.figure(figsize=(6, 12))  # 调整图像大小使其看起来修长
    for df, label in zip(dfs, labels):
        if len(df) > 0:
            plt.plot(df['tx'], df['ty'], label=label)
    plt.title('XY Trajectory')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.xticks(ticks=np.arange(min(dfs[0]['tx']), max(dfs[0]['tx']), step=0.5))  # 调整X坐标间隔
    plt.yticks(ticks=np.arange(min(dfs[0]['ty']), max(dfs[0]['ty']), step=1.0))  # 调整Y坐标间隔
    plt.grid(False)
    plt.legend()
    plt.show()

def plot_position_components(dfs, labels):
    plt.figure(figsize=(15, 8))

    for i, (df, label) in enumerate(zip(dfs, labels)):
        if len(df) > 0:
            df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]

            plt.subplot(3, 1, 1)
            plt.plot(df['relative_time'], df['tx'], label=label)
            plt.title('X Component over Time')
            plt.xlabel('Time (s)')
            plt.ylabel('X')
            plt.grid(True)
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.plot(df['relative_time'], df['ty'], label=label)
            plt.title('Y Component over Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Y')
            plt.grid(True)
            plt.legend()

            plt.subplot(3, 1, 3)
            plt.plot(df['relative_time'], df['tz'], label=label)
            plt.title('Z Component over Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Z')
            plt.grid(True)
            plt.legend()

    plt.tight_layout()
    plt.show()

def plot_velocity_components(dfs, labels):
    plt.figure(figsize=(15, 8))

    for i, (df, label) in enumerate(zip(dfs, labels)):
        if len(df) > 0:
            df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]

            plt.subplot(3, 1, 1)
            plt.plot(df['relative_time'], df['vx'], label=label)
            plt.title('X Velocity over Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Vx')
            plt.grid(True)
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.plot(df['relative_time'], df['vy'], label=label)
            plt.title('Y Velocity over Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Vy')
            plt.grid(True)
            plt.legend()

            plt.subplot(3, 1, 3)
            plt.plot(df['relative_time'], df['vz'], label=label)
            plt.title('Z Velocity over Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Vz')
            plt.grid(True)
            plt.legend()

    plt.tight_layout()
    plt.show()

def plot_rpy_components(dfs, labels):
    plt.figure(figsize=(15, 8))

    for df, label in zip(dfs, labels):
        if 'yaw' in df.columns and len(df) > 0 and label != 'WVO Pose':
            df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]

            plt.plot(df['relative_time'], df['yaw'], label=label)
    
    plt.title('Yaw Angle over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (degrees)')
    plt.grid(True)
    plt.legend()
    plt.show()

def main():
    file_paths = ['wo_pose.tum', 'wio_pose.tum', 'wvo_pose.tum', 'state_pose.tum']
    labels = ['WO Pose', 'WIO Pose', 'WVO Pose', 'State Pose']

    dfs = []
    for file_path, label in zip(file_paths, labels):
        df = read_tum_data(file_path)
        if len(df) > 0:
            df = compute_velocity(df)
            if label not in ['WVO Pose', 'State Pose']:  # 过滤掉 wvo_pose 和 state_pose
                df = compute_rpy(df)
        dfs.append(df)

    plot_trajectory(dfs, labels)
    plot_position_components(dfs, labels)
    plot_velocity_components(dfs, labels)
    plot_rpy_components(dfs, labels)

if __name__ == "__main__":
    main()
