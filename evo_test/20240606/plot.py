import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

def read_tum_data(file_path):
    columns = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
    df = pd.read_csv(file_path, delim_whitespace=True, header=None, names=columns)
    return df

def compute_velocity(df):
    df['dt'] = df['timestamp'].diff()
    df['vx'] = df['tx'].diff() / df['dt']
    df['vy'] = df['ty'].diff() / df['dt']
    df['vz'] = 0.0
    return df

def plot_trajectory(df):
    plt.figure(figsize=(10, 6))
    plt.plot(df['tx'], df['ty'], color='black')
    plt.title('XY Trajectory')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.show()

def plot_position_components(df):
    # Convert timestamp to seconds relative to the start time
    df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]

    plt.figure(figsize=(15, 8))

    plt.subplot(3, 1, 1)
    plt.plot(df['relative_time'], df['tx'], color='blue')
    plt.title('X Component over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('X')

    plt.subplot(3, 1, 2)
    plt.plot(df['relative_time'], df['ty'], color='green')
    plt.title('Y Component over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Y')

    plt.subplot(3, 1, 3)
    plt.plot(df['relative_time'], df['tz'], color='red')
    plt.title('Z Component over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Z')

    plt.tight_layout()
    plt.show()

def plot_velocity_components(df):
    # Convert timestamp to seconds relative to the start time
    df['relative_time'] = df['timestamp'] - df['timestamp'].iloc[0]

    plt.figure(figsize=(15, 8))

    plt.subplot(3, 1, 1)
    plt.plot(df['relative_time'], df['vx'], color='blue')
    plt.title('X Velocity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Vx')

    plt.subplot(3, 1, 2)
    plt.plot(df['relative_time'], df['vy'], color='green')
    plt.title('Y Velocity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Vy')

    plt.subplot(3, 1, 3)
    plt.plot(df['relative_time'], df['vz'], color='red')
    plt.title('Z Velocity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Vz')

    plt.tight_layout()
    plt.show()

def main():
    file_path = 'wo_sync.txt'
    df = read_tum_data(file_path)
    df = compute_velocity(df)
    plot_trajectory(df)
    plot_position_components(df)
    plot_velocity_components(df)

if __name__ == "__main__":
    main()