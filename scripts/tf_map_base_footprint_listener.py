#!/usr/bin/env python

import os
import csv
import rospy
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('tf_map_base_listener', anonymous=False)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # определение пути к директории CATKIN_WORKSPACE
    workspace_path = os.environ['CATKIN_WORKSPACE']
    file_path = os.path.join(workspace_path, 'src/slam_algorithm_comparasion/tf_map_base_listener.csv')
    # создайте файл csv и запишите заголовки столбцов
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y'])

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))

            # откройте файл csv и добавьте новые строки данных
            with open(workspace_path + '/src/slam_algorithm_comparasion/tf_map_base_listener.csv', mode='a') as file:
                writer = csv.writer(file)
                writer.writerow([trans.transform.translation.x, trans.transform.translation.y])

            rate.sleep()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue