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
from tf.transformations import euler_from_quaternion

def trajectory_callback(trajectory_msg : TrajectoriesStamped, writer):
    
    secs = trajectory_msg.header.stamp.secs
    nsecs = trajectory_msg.header.stamp.nsecs

    for trajectory in trajectory_msg.trajectories:
        row = []

        id = trajectory.id
        curr_x_pos = trajectory.current.pose.pose.position.x
        current_y_pos = trajectory.current.pose.pose.position.y

        quat_z = trajectory.current.pose.pose.orientation.z
        quat_w = trajectory.current.pose.pose.orientation.w
        
        quaternion = (0.0, 0.0, quat_z, quat_w)
        euler_angles = euler_from_quaternion(quaternion)
        yaw = euler_angles[2]

        x_yaw = np.cos(yaw)
        y_yaw = np.sin(yaw)

        row.append(secs)
        row.append(nsecs)
        row.append(id)
        row.append(curr_x_pos)
        row.append(current_y_pos)
        row.append(x_yaw)
        row.append(y_yaw)

        writer.writerow(row)

    print("Done row something")


if __name__ == "__main__":

    rospy.init_node('tracks_write', anonymous=True)
    t = time.localtime()
    current_time = time.strftime("%H:%M:%S", t)

    output_file = "csv_tracks" + current_time
    
    with open(output_file, "w") as f:
        writer = csv.writer(f)

        header = ['secs','nsecs','id', 'x_pos', 'y_pos', 'x_yaw', 'y_yaw']
        writer.writerow(header)
        rospy.Subscriber("/pedestrians_tracker/objects_trajectories", TrajectoriesStamped, lambda msg: trajectory_callback(msg, writer), queue_size=100)    
        rospy.spin()