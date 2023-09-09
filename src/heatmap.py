import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from scipy.stats import binned_statistic_2d
def build_heatmap(
    csv_directory, x_pos_label, y_pos_label, cell_size = 0.5, noise_thresh = 20
):
    x_positions = []
    y_positions = []
    vx_velocities = []
    vy_velocities = []
    for filename in os.listdir(csv_directory):
        f = os.path.join(csv_directory, filename)

        if (os.path.isfile(f)) and ".csv" in f:
            df1 = pd.read_csv(f)
            x_pos = np.array(df1[x_pos_label])
            y_pos = np.array(df1[y_pos_label])

            for elem in x_pos:
                x_positions.append(elem)

            for elem in y_pos:
                y_positions.append(elem)

    x_bins = int((max(x_positions) - min(x_positions)) / cell_size)
    y_bins = int((max(y_positions) - min(y_positions)) / cell_size)
    # Plot the positions and biased velocities


    # Calculate 2D histogram
    H, x_edges, y_edges, bin_numbers = binned_statistic_2d(
        x_positions, y_positions, None, statistic="count", bins=[x_bins, y_bins]
    )

    # Threshold the heatmap based on intensity
    H_thresholded = np.where(H <= noise_thresh, H, 0)

    # Plot the heatmap
    fig, ax = plt.subplots(figsize=(8, 6))
    plt.imshow(H_thresholded.T, extent=[x_edges[0], x_edges[-1], y_edges[0], y_edges[-1]], cmap=plt.cm.jet)
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title("Pedestrian positions")
    # Add colorbar with a label
    cbar = plt.colorbar()
    cbar.ax.set_ylabel('Intensity')
    # plt.show()


    # plt.show()
