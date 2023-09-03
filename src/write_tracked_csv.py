import csv 
import numpy as np
import rospy
from zio_obstacle_msgs.msg import (
    ObjectsSequenceStamped,
    ObjectsStamped,
    Object,
    Obstacle,
    TrajectoriesStamped,
    Trajectory
)
import time

def trajectory_callback(trajectory_msg, writer):
    
    for trajectory in trajectory_msg.trajectories:
        row = []

        id = trajectory.id
        curr_x_pos = trajectory.current.pose.pose.position.x
        current_y_pos = trajectory.current.pose.pose.position.y

        row.append(id)
        row.append(curr_x_pos)
        row.append(current_y_pos)

        writer.writerow(row)

    print("Done row something")


if __name__ == "__main__":

    rospy.init_node('tracks_write', anonymous=True)
    t = time.localtime()
    current_time = time.strftime("%H:%M:%S", t)

    output_file = "csv_tracks" + current_time
    
    with open(output_file, "w") as f:
        writer = csv.writer(f)

        header = ['id', 'x_pos', 'y_pos']
        writer.writerow(header)
        rospy.Subscriber("/pedestrians_tracker/objects_trajectories", TrajectoriesStamped, lambda msg: trajectory_callback(msg, writer), queue_size=100)    
        rospy.spin()