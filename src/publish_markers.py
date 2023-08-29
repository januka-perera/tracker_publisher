#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import argparse
import pandas as pd
from random import randint

class MarkerPublisher:
    def __init__(self, file_name):
        rospy.init_node('marker_publisher')
        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray)
        self.markers = []
        self.file_name = file_name

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.id = randint(1,1000000)
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.lifetime = rospy.Duration(2)        

        self.markers.append(marker)

    def publish_marker_array(self):
        marker_array = MarkerArray()
        marker_array.markers = self.markers

        self.marker_pub.publish(marker_array)
        
    def spin(self):
        rospy.spin()

    def publish_frame_data(self):

        df = pd.read_csv(self.file_name)

        frame_id = 0

        rospy.loginfo("Starting publishing frames")

        for index, row in df.iterrows():

            current_frame = int(row["Frame#"])

            if current_frame != frame_id:
                self.publish_marker_array()

                frame_id += 1
                self.markers = []

            x = row["x"]
            y = row["y"]

            print(x,y)
            self.publish_marker(x,y)
            rospy.sleep(0.001)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="Enter the absolute file path", type=str)
    args = parser.parse_args()

    file_name = args.path
    marker_publisher = MarkerPublisher(file_name)
    marker_publisher.publish_frame_data()
    marker_publisher.spin()