# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import pandas as pd
# import argparse
# import rospy
# from visualization_msgs.msg import Marker, MarkerArray


# class Dynamic_Plotter:

#     def __init__(self, raw_data):
#         self.raw_data = raw_data
#         pass

#     def raw_marker_plotter(self):
#         print("Starting publishing frames")

#         x_array = []
#         y_array = []
#         for index, row in self.raw_data.iterrows():
#             current_frame = int(row["Frame#"])

#             if current_frame != frame_id:
                
#                 # Then plot all the points in x_array, y_array

#                 frame_id += 1
#                 self.markers = []

#             x = row["x"]
#             y = row["y"]

#             print(x,y)
#             x_array.append(x)
#             y_array.append(y)
            
#         pass

        



# if __name__ == "__main__":

#     parser = argparse.ArgumentParser()
#     parser.add_argument("path", help="Enter the absolute file path", type=str)
#     args = parser.parse_args()

#     file_name = args.path

#     raw_data = pd.read_csv(file_name)


#     all_plotters = Dynamic_Plotter(raw_data)
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import argparse

class Dynamic_Plotter:
    def __init__(self, raw_data):
        self.raw_data = raw_data
        self.fig, self.ax = plt.subplots()
        self.x_array = []
        self.y_array = []
        self.scatter = self.ax.scatter([], [], c='b', marker='o')  # Initialize an empty scatter plot
        self.ax.set_xlim(0, 100)  # Adjust x-axis limits as needed
        self.ax.set_ylim(0, 100)  # Adjust y-axis limits as needed

        self.max_data_points = 500

        self.ani = FuncAnimation(self.fig, self.update_plot, frames=len(self.raw_data), blit=True, interval=1)

    def update_plot(self, frame):
        row = self.raw_data.iloc[frame]
        x = row["x"]
        y = row["y"]

        self.x_array.append(x)
        self.y_array.append(y)

        if len(self.x_array) > self.max_data_points:
            self.x_array.pop(0)
            self.y_array.pop(0)

        self.scatter.set_offsets(np.column_stack((self.x_array, self.y_array)))

        return self.scatter,

    def show_animation(self):
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="Enter the absolute file path", type=str)
    args = parser.parse_args()

    file_name = args.path
    raw_data = pd.read_csv(file_name)

    plotter = Dynamic_Plotter(raw_data)
    plotter.show_animation()
