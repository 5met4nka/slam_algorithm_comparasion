import os
import csv
import matplotlib.pyplot as plt

def read_csv(file_path):
    lidar_std = []
    slam_std = []

    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            lidar_std.append(float(row[0]))
            slam_std.append(float(row[1]))

    return lidar_std, slam_std

def plot_std_comparison(lidar_std, slam_std, label):
    plt.plot(
            lidar_std,
            slam_std,
            label=label
            )

def show_plot():
    plt.xlabel('Lidar STD')
    plt.ylabel('SLAM STD')
    plt.title('STD Comparison')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    workspace_path = os.environ['CATKIN_WORKSPACE']
    hector_csv_file = os.path.join(workspace_path, 'src/slam_algorithm_comparasion/turlebot3_stage_4/hector_mapping.csv')
    slam_csv_file = os.path.join(workspace_path, 'src/slam_algorithm_comparasion/turlebot3_stage_4/slam_gmapping.csv')

    lidar_std_hector, slam_std_hector = read_csv(hector_csv_file)
    plot_std_comparison(lidar_std_hector, slam_std_hector, 'hector_mapping')

    lidar_std_slam, slam_std_slam = read_csv(slam_csv_file)
    plot_std_comparison(lidar_std_slam, slam_std_slam, 'slam_gmapping')

    show_plot()