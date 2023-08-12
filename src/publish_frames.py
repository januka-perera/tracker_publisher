#!/usr/bin/env python3

import rospy
from tf.transformations import quaternion_from_euler
import argparse
from std_msgs.msg import String, Header
from zio_obstacle_msgs.msg import (
    ObjectsSequenceStamped,
    ObjectsStamped,
    Object,
    Obstacle,
)

from geometry_msgs.msg import (
    PoseWithCovariance,
    TwistWithCovariance,
    AccelWithCovariance,
    Polygon,
    Point32,
)

from shape_msgs.msg import SolidPrimitive

import pandas as pd
import numpy as np


def get_bounding_box_coordinates(box_params):
    center = box_params[:3]  # Extract center (x, y, z)
    size = box_params[3:6]  # Extract size (length, width, height)
    rotation = box_params[6]  # Extract rotation (yaw angle)

    # Calculate half sizes in each dimension
    half_length = size[0] / 2
    half_width = size[1] / 2
    half_height = size[2] / 2

    # Define the 8 corners of the bounding box in local coordinates
    corners_local = np.array(
        [
            [-half_length, -half_width, -half_height],
            [half_length, -half_width, -half_height],
            [half_length, half_width, -half_height],
            [-half_length, half_width, -half_height],
            [-half_length, -half_width, half_height],
            [half_length, -half_width, half_height],
            [half_length, half_width, half_height],
            [-half_length, half_width, half_height],
        ]
    )

    # Apply rotation to the local coordinates to get the coordinates in the global frame
    rotation_matrix = np.array(
        [
            [np.cos(rotation), -np.sin(rotation), 0],
            [np.sin(rotation), np.cos(rotation), 0],
            [0, 0, 1],
        ]
    )

    # Calculate the coordinates of the bounding box vertices in the global frame
    corners_global = np.dot(corners_local, rotation_matrix.T) + center

    return corners_global


def format_pose(pedestrian: Object, data):
    pedestrian.pose.pose.position.x = data["x"]
    pedestrian.pose.pose.position.y = data["y"]
    pedestrian.pose.pose.position.z = data["z"]

    quat = quaternion_from_euler(1e-10, 1e-10, data["orientation"])

    pedestrian.pose.pose.orientation.x = quat[0]
    pedestrian.pose.pose.orientation.y = quat[1]
    pedestrian.pose.pose.orientation.z = quat[2]
    pedestrian.pose.pose.orientation.w = quat[3]


def format_bounding_box(pedestrian: Object, data):
    pedestrian.shape.type = pedestrian.shape.BOX

    pedestrian.shape.dimensions.append(data["l"])
    pedestrian.shape.dimensions.append(data["w"])
    pedestrian.shape.dimensions.append(data["h"])


def format_birds_eye_view(pedestrian: Object, data):
    x = data["x"]
    y = data["y"]
    z = data["z"]

    l = data["l"]
    w = data["w"]
    h = data["h"]

    yaw = data["orientation"]

    global_coords = get_bounding_box_coordinates([x, y, z, l, w, h, yaw])

    # The shape boundary will be defined by 4 coordinates in top-down view

    for i in range(4):
        new_point = Point32()
        new_point.x = global_coords[i][0]
        new_point.y = global_coords[i][1]
        new_point.z = 0.0

        pedestrian.shape_boundary.points.append(new_point)


def format_timestamp_header(header: Header, secs, nsecs, id):
    header.frame_id = id
    header.seq = id

    header.stamp.secs = int(secs)
    header.stamp.nsecs = int(nsecs)


class TrackerPublisher:
    def __init__(self, file_path):
        print(f"Path to file is {file_path}")
        self.file_path = file_path

        self.ObjectsSequenceStamped_publisher = rospy.Publisher(
            "/objects_data", ObjectsSequenceStamped, queue_size=10
        )
        self.ObjectsSequenceStamped_msg = ObjectsSequenceStamped()

        self.PedestrianObjects = []

        self.current_secs = None
        self.current_nsecs = None

    def format_msg_header(self, data, id):
        self.ObjectsSequenceStamped_msg.header.stamp.secs = int(data["secs"])
        self.ObjectsSequenceStamped_msg.header.stamp.nsecs = int(data["nsecs"])
        self.ObjectsSequenceStamped_msg.header.frame_id = id
        self.ObjectsSequenceStamped_msg.header.seq = id

    def publish_frame_data(self):
        df = pd.read_csv(self.file_path)

        frame_id = 0
        for index, row in df.iterrows():
            current_frame = int(row["Frame#"])

            # If this a new frame data from the previous frame needs to be published
            if current_frame != frame_id:
                objects_stamped_msg = ObjectsStamped()
                objects_stamped_msg.objects = self.PedestrianObjects
                format_timestamp_header(
                    objects_stamped_msg.header,
                    self.current_secs,
                    self.current_nsecs,
                    frame_id,
                )

                format_timestamp_header(
                    self.ObjectsSequenceStamped_msg.header,
                    self.current_secs,
                    self.current_nsecs,
                    frame_id,
                )

                self.ObjectsSequenceStamped_msg.source_id = frame_id
                self.ObjectsSequenceStamped_msg.objects_sequence = objects_stamped_msg

                self.ObjectsSequenceStamped_publisher.publish(
                    self.ObjectsSequenceStamped_msg
                )

                print(self.ObjectsSequenceStamped_msg)

                frame_id += 1
                self.PedestrianObjects = []

            # Construct the pedestrian object
            pedestrian_object = Object()

            pedestrian_object.id = index
            pedestrian_object.category = "Person"
            pedestrian_object.confidence = row["confidence_level"]

            format_pose(pedestrian_object, row)
            format_bounding_box(pedestrian_object, row)
            format_birds_eye_view(pedestrian_object, row)

            self.PedestrianObjects.append(pedestrian_object)

            self.current_secs = row["secs"]
            self.current_nsecs = row["nsecs"]


def main():
    rospy.init_node("publish_frames")

    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="Enter the absolute file path", type=str)
    args = parser.parse_args()

    file_name = args.path

    pedestrian_tracker = TrackerPublisher(file_name)
    pedestrian_tracker.publish_frame_data()
    print(file_name)

    rospy.spin()


if __name__ == "__main__":
    main()
