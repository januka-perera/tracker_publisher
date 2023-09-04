import csv 
import numpy as np
import argparse
import pandas as pd

import matplotlib.pyplot as plt

from matplotlib.colors import ListedColormap
from matplotlib import colors as mcolors


#!/usr/bin/env python


class Static_Plotter:
    def __init__(self, raw_data):
        self.raw_data = raw_data
        self.all_x = raw_data["x_pos"]
        self.all_y = raw_data["y_pos"]
        self.all_ids = raw_data["id"]

        self.all_colors = list(mcolors.CSS4_COLORS.keys())
        n = (len(self.all_colors)) // 140
        print(n)
        self.distinct_colours = self.all_colors[::n]
        self.custom_colour_map = ListedColormap(self.distinct_colours)


        self.unique_ids = pd.unique(self.all_ids).tolist()
        print(self.unique_ids)
    def scatter_plot(self):
       plt.scatter(self.all_x, self.all_y, cmap  = self.custom_colour_map, c = [self.unique_ids.index(elem) for elem in self.all_ids])
       plt.show()


if __name__ == "__main__":
    print("x")
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="Enter the absolute file path", type=str)
    args = parser.parse_args()
    
    file_name = args.path
    raw_data = pd.read_csv(file_name)

    # print(raw_data["id"])
    print(raw_data["id"].nunique())
    
    XY_plotter = Static_Plotter(raw_data=raw_data)
    XY_plotter.scatter_plot()

    

    