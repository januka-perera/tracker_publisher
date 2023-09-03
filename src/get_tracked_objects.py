#!/usr/bin/env python

import rospy

import argparse
from std_msgs.msg import String, Header
from zio_obstacle_msgs.msg import (
    ObjectsSequenceStamped,
    ObjectsStamped,
    Object,
    Obstacle,
    TrajectoriesStamped,
    Trajectory
)

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

class Plotter:

    def __init__(self):

        self.fig, self.ax = plt.subplots()
        self.scatter = self.ax.scatter([], [], c=[], cmap=plt.get_cmap('plasma'))

        self.ax.set_xlim(-80,80)
        self.ax.set_ylim(-40,70)

        self.x_data = []
        self.y_data = []
        self.frame_x = []
        self.frame_y = []
        self.frame_ids = []
        self.ids = []
        self.global_ids = []
        self.max_data_points = 50

        self.ani = FuncAnimation(self.fig, self.update_plot, blit=True, interval=0.01, frames= 20000)  # Adjust interval as needed

    def map_to_ids(self):
        id_mapping = {}
        id_list = []
        next_id = 0

        for num in self.ids:
            if num not in id_mapping:
                id_mapping[num] = next_id
                next_id += 1
            id_list.append(id_mapping[num])

        return id_list

    def update_plot(self,frame):
        
        if len(self.x_data) > self.max_data_points:
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.ids.pop(0)
    
        self.scatter = self.ax.scatter(self.x_data, self.y_data, c = [self.global_ids.index(elem) for elem in self.ids], cmap=plt.get_cmap('plasma'))

        return self.scatter,



    def show_animation(self):
        plt.show()

class TrackedPerson:
    def __init__(self):
        self.id = None
        self.current_x = None
        self.current_y = None

        self.prev_x = []
        self.prev_y = []
        
       




def trajectory_callback(trajectory_msg, plotter):
    i = 0
    for trajectory in trajectory_msg.trajectories:
        person = TrackedPerson()
        id = trajectory.id
        curr_x_pos = trajectory.current.pose.pose.position.x
        current_y_pos = trajectory.current.pose.pose.position.y

        person.id = id
        person.current_x = curr_x_pos
        person.current_y = current_y_pos

        plotter.frame_ids.append(id)
        plotter.frame_x.append(curr_x_pos)
        plotter.frame_y.append(current_y_pos)

        if id not in plotter.global_ids:
            plotter.global_ids.append(id)

        print("id = {}, x_pos = {}, y_pos = {}".format(id, curr_x_pos, current_y_pos))
        
        i += 1
        print(i)
    
    plotter.ids.extend(plotter.frame_ids)
    plotter.x_data.extend(plotter.frame_x)
    plotter.y_data.extend(plotter.frame_y)


    plotter.frame_ids = []
    plotter.frame_x = []
    plotter.frame_y = [] 

        
    


if __name__ == "__main__":
    rospy.init_node('tracks_sub', anonymous=True)
    plotter = Plotter()
    rospy.Subscriber("/pedestrians_tracker/objects_trajectories", TrajectoriesStamped, lambda msg: trajectory_callback(msg, plotter), queue_size=100)
    plotter.show_animation()
    rospy.spin()