#!/usr/bin/env python

import os
import csv
import rospy
from nav_msgs.msg import Odometry

def callback_pose(data, writer):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    writer.writerow([x, y])

def listener():
    rospy.init_node('ground_truth_listener', anonymous=False)
    workspace_path = os.environ['CATKIN_WORKSPACE']
    file_path = os.path.join(workspace_path, 'src/slam_algorithm_comparasion/ground_truth_listener.csv')
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y'])  # записываем заголовок файла CSV
        rospy.Subscriber('/ground_truth/state', Odometry, callback_pose, writer, queue_size=10)
        rospy.spin()

if __name__ == '__main__':
    listener()