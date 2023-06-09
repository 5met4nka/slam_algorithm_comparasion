#!/usr/bin/env python

import os
import csv
import math
import matplotlib.pyplot as plt
import numpy as np

def read_csv(filename):
    data = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # пропускаем заголовок
        for row in reader:
            data.append([float(row[0]), float(row[1])])
    return data

def calculate_std_dev(slam_data_x, slam_data_y, ground_truth_data_x, ground_truth_data_y):
    # вычисление разницы между данными ground truth и данными SLAM для X и Y
    differences_x = []
    differences_y = []
    for i in range(len(ground_truth_data_x)):
        diff_x = slam_data_x[i] - ground_truth_data_x[i]
        differences_x.append(diff_x)
        diff_y = slam_data_y[i] - ground_truth_data_y[i]
        differences_y.append(diff_y)
    # вычисление среднее значение разницы для X и Y
    mean_x = sum(differences_x) / len(differences_x)
    mean_y = sum(differences_y) / len(differences_y)
    # вычисление сумму квадратов разностей для X и Y
    ssd_x = sum([(x - mean_x) ** 2 for x in differences_x])
    ssd_y = sum([(y - mean_y) ** 2 for y in differences_y])
    # вычисление СКО для X и Y
    std_dev_x = math.sqrt(ssd_x / (len(differences_x) - 1))
    std_dev_y = math.sqrt(ssd_y / (len(differences_y) - 1))
    return std_dev_x, std_dev_y

def main():
    # задаем имена файлов в домашней директории
    workspace_path = os.environ['CATKIN_WORKSPACE']
    ground_truth_file = os.path.join(workspace_path, 'src/slam_algorithm_comparasion/ground_truth_listener.csv')
    slam_data_file = os.path.join(workspace_path, 'src/slam_algorithm_comparasion/tf_map_base_listener.csv')

    # читаем данные из файлов
    ground_truth_data = read_csv(ground_truth_file)
    slam_data = read_csv(slam_data_file)

    # разделение координат X и Y
    slam_data_x = [x[0] for x in slam_data]
    slam_data_y = [x[1] for x in slam_data]
    ground_truth_data_x = [x[0] for x in ground_truth_data]
    ground_truth_data_y = [x[1] for x in ground_truth_data]

    # вычисление значений СКО для X и Y
    std_dev_x, std_dev_y = calculate_std_dev(slam_data_x, slam_data_y, ground_truth_data_x, ground_truth_data_y)

    # вывод значений СКО для X и Y
    print("Standard deviation for X:", std_dev_x)
    print("Standard deviation for Y:", std_dev_y)

    # --------------------------------------------------------------------------------------------------------------
    # построение данных для ground truth и SLAM

    errors = [(abs(slam_data_x[i] - ground_truth_data_x[i]), abs(slam_data_y[i] - ground_truth_data_y[i])) for i in range(len(slam_data_x))]
    colors = [(((error[0])**2 + (error[1])**2)**0.5) for error in errors]

    plt.plot(
            ground_truth_data_x,
            ground_truth_data_y,
            linestyle='-',
            color='black',
            label='Ground Truth Data'
            )
    
    plt.scatter(
                slam_data_x,
                slam_data_y,
                marker='.',
                c=colors,
                cmap='RdYlGn_r',
                label='SLAM Data'
                )
    plt.colorbar()

    plt.legend()
    plt.xlabel('X coord, meters')
    plt.ylabel('Y coord, meters')
    plt.grid()
    plt.title('Coordinate Data')
    plt.show()

    # --------------------------------------------------------------------------------------------------------------
    # построение данных по координате X для ground truth и SLAM

    colors = [(error[0]) for error in errors]

    plt.xlim(0, len(ground_truth_data_x))
    plt.plot(
            ground_truth_data_x,
            linestyle='-',
            color='black',
            label='Ground Truth Data'
            )

    plt.scatter(
                range(len(slam_data_x)),
                slam_data_x,
                marker='.',
                c=colors,
                cmap='RdYlGn_r',
                label='SLAM Data'
                )
    plt.colorbar()

    plt.legend()
    plt.ylabel('X coord, meters')
    plt.grid()
    plt.title('X Coordinate Data')
    plt.show()

    # --------------------------------------------------------------------------------------------------------------
    # построение данных по координате Y для ground truth и SLAM

    colors = [(error[1]) for error in errors]

    plt.xlim(0, len(ground_truth_data_y))
    plt.plot(
            ground_truth_data_y,
            linestyle='-',
            color='black',
            label='Ground Truth Data'
            )

    plt.scatter(
                range(len(slam_data_y)),
                slam_data_y,
                marker='.',
                c=colors,
                cmap='RdYlGn_r',
                label='SLAM Data'
                )
    plt.colorbar()

    plt.legend()
    plt.ylabel('Y coord, meters')
    plt.grid()
    plt.title('Y Coordinate Data')
    plt.show()

    # --------------------------------------------------------------------------------------------------------------
    # построение графика отклонения SLAM от Ground Truth для X

    x_error = [(gt - slam) for gt, slam in zip(ground_truth_data_x, slam_data_x)]
    colors = [(error[0]) for error in errors]

    plt.xlim(0, len(x_error))
    plt.ylim(-0.5, 0.5)
    plt.plot(
            [0, len(x_error)],
            [0, 0],
            linestyle='--',
            color='black',
            label='Expected value'
            )

    plt.scatter(
                range(len(x_error)),
                x_error,
                marker='.',
                c=colors,
                cmap='RdYlGn_r',
                label='X Error'
                )
    plt.colorbar()

    x_std_dev = np.std(x_error)
    plt.fill_between(
                    range(len(x_error)),
                    [x_std_dev]*len(x_error),
                    [-x_std_dev]*len(x_error),
                    alpha=0.3,
                    color='black',
                    label='STD'
                    )

    plt.legend()
    plt.ylabel('X coord SLAM error, meters')
    plt.grid()
    plt.title('SLAM Error (X Coordinate)')
    plt.show()

    # --------------------------------------------------------------------------------------------------------------
    # построение графика отклонения SLAM от Ground Truth для Y

    y_error = [(gt - slam) for gt, slam in zip(ground_truth_data_y, slam_data_y)]
    colors = [(error[1]) for error in errors]

    plt.xlim(0, len(y_error))
    plt.ylim(-0.5, 0.5)
    plt.plot(
            [0, len(x_error)],
            [0, 0],
            linestyle='--',
            color='black',
            label='Expected value'
            )

    plt.scatter(
                range(len(y_error)),
                y_error,
                marker='.',
                c=colors,
                cmap='RdYlGn_r',
                label='Y Error'
                )
    plt.colorbar()

    y_std_dev = np.std(y_error)
    plt.fill_between(
                    range(len(y_error)),
                    [y_std_dev]*len(y_error),
                    [-y_std_dev]*len(y_error),
                    alpha=0.3,
                    color='black',
                    label='STD'
                    )

    plt.legend()
    plt.ylabel('Y coord SLAM error, meters')
    plt.grid()
    plt.title('SLAM Error (Y Coordinate)')
    plt.show()

if __name__ == '__main__':
    main()