from heatmap import build_heatmap
from prettify_plot import prettify_plot
from vector_field import draw_vector_plot
import time
if __name__ == "__main__":
    dataset_directory = "2d_csvs"
    t = time.localtime()
    current_time = time.strftime("%H:%M:%S", t)
    build_heatmap(csv_directory=dataset_directory, x_pos_label="x_pos", y_pos_label="y_pos", cell_size=0.5)
    prettify_plot("Heatmap Sep 9 Lab" +current_time)
    # draw_vector_plot(dataset_directory, "x_pos", "y_pos", "x_yaw", "y_yaw")
    
    # prettify_plot("VectorFieldFirstIteration"+current_time)
