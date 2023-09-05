import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
import pandas as pd
from sklearn.mixture import GaussianMixture
from matplotlib.lines import Line2D
import os
from math import ceil, floor


def is_gap_too_small(arr, tolerance = 10):
    for i in range(len(arr)):
        for j in range(i+1, len(arr)):
            if abs(arr[i][0] - arr[j][0]) < tolerance:
                return True
    return False


def draw_vector_plot(
    dataset_directory, x_pos_label, y_pos_label, x_rot_label, y_rot_label, cell_size=0.5
):
    x_positions = []
    y_positions = []
    x_rotations = []
    y_rotations = []
    for filename in os.listdir(dataset_directory):
        f = os.path.join(dataset_directory, filename)

        if (os.path.isfile(f)) and ".csv" in f:
            df1 = pd.read_csv(f)
            x_pos = np.array(df1[x_pos_label])
            y_pos = np.array(df1[y_pos_label])
            x_rot = np.array(df1[x_rot_label])
            y_rot = np.array(df1[y_rot_label])

            for elem in x_pos:
                x_positions.append(elem)

            for elem in y_pos:
                y_positions.append(elem)

            for elem in x_rot:
                x_rotations.append(elem)

            for elem in y_rot:
                y_rotations.append(elem)

    print(max(y_positions))

    cell_width = cell_height = cell_size

    map_width = int((ceil(max(x_positions) - min(x_positions)) + 1) / cell_width)
    map_height = int((ceil(max(y_positions) - min(y_positions)) + 1) / cell_height)

    map_left = floor(min(x_positions))
    map_bottom = floor(min(y_positions))
    map_top = ceil(max(y_positions))

    print(map_width, map_height, map_left, map_bottom, map_top)

    rotations = np.empty((map_height, map_width), dtype=object)
    # velocities = np.zeros((map_width, map_height, 2))
    for i in range(len(x_positions)):
        x_cell = int((x_positions[i] - map_left) // cell_width)
        y_cell = int((y_positions[i] - map_bottom) // cell_height)
        # print(x_positions[i], y_positions[i])
        # print(x_cell, y_cell)

        if rotations[y_cell, x_cell] == None:
            rotations[y_cell, x_cell] = []

        rotations_vector = np.array([x_rotations[i], y_rotations[i]])

        xvel = rotations_vector[0]
        yvel = rotations_vector[1]
        angle = np.rad2deg(np.arctan2(yvel, xvel))
        rotations[y_cell, x_cell].append(angle)

    print((rotations[2, 4]))

    gmm = GaussianMixture(n_components=1)

    gaussian_angles = np.empty((map_height, map_width), dtype=object)

    cell_directions_range = range(1,4)

    for y in range(rotations.shape[0]):
        for x in range(rotations.shape[1]):
            
            if (rotations[y,x] != None):
                current_cell = np.array(rotations[y,x]).reshape(-1,1)
                
                if len(current_cell) > 1:
                    models = [GaussianMixture(n_components=n).fit(current_cell) for n in cell_directions_range if n < len(current_cell)]
                    bic_scores = [m.bic(current_cell) for m in models]
                    lowest_bic_score = np.argmin(bic_scores)
                    # print("hello")
                    # print(current_cell)
                    # print("----------------------------------------")
                    # print(models[lowest_bic_score].means_)
                    # print(y,x)

                    gaussian_angles[y,x] = models[lowest_bic_score]

                    # print("Original Modes")
                    
                    # print(gaussian_angles[y,x].means_)

                    while (is_gap_too_small(gaussian_angles[y,x].means_, tolerance=15) == True):
                        gaussian_angles[y,x] = models[lowest_bic_score-1]
                        lowest_bic_score -= 1


    mean_directions = np.empty((map_height, map_width), dtype=object)

    for y in range(gaussian_angles.shape[0]):
        for x in range(gaussian_angles.shape[1]):

            if (gaussian_angles[y,x] != None):
                num_peaks = gaussian_angles[y,x].n_components
                mean_angles = [np.deg2rad(mean_angle)for mean_angle in gaussian_angles[y,x].means_]
                if num_peaks == 3:
                    print(y,x)
                    print(mean_angles)
                
                if mean_directions[y,x] == None:
                    mean_directions[y,x] = []

                for angle in mean_angles:
                    x_vel = np.cos(angle)
                    y_vel = np.sin(angle)

                    mean_directions[y,x].append([x_vel, y_vel])


    fig, ax = plt.subplots()
    for y in range(mean_directions.shape[0]):
        for x in range(mean_directions.shape[1]):
            vectors = mean_directions[y,x]
            if vectors is not None:
                for vector in vectors:
                    vx, vy = vector
                    # print(x,y)
                    color = ''
                    if len(vectors) == 3:
                        color = 'r'

                    elif len(vectors) == 2:
                        color = 'g'

                    else:
                        color = 'b'
                        

                    
                    ax.quiver(x*cell_width + min(x_positions), y*cell_height + min(y_positions), vx, vy, color=color, scale=2, units='xy', angles='xy', width=0.02, headwidth=3.5, headlength=4, headaxislength=3)

    # set the axis limits and show the plot
    ax.set_xlim(min(x_positions)-1, max(x_positions)+1)
    ax.set_ylim(min(y_positions)-1, max(y_positions)+1)
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title("Vector Field with Gaussian fitting for direction")

    legend_elements = [
        Line2D([0], [0], color='b', lw=1, label='1 Mode'),
        Line2D([0], [0], color='g', lw=1, label='2 Modes'),
        Line2D([0], [0], color='r', lw=1, label='3 Modes')
    ]

    # Add the legend to the plot
    ax.legend(handles=legend_elements, loc='upper right', fontsize = 12)
    