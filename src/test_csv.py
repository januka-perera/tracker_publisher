import pandas as pd


file_name = "/home/januka/tracking_catkin/src/tracker_publisher/src/3d_bounding1.txt"

df = pd.read_csv(file_name)

i = 0
for index, row in df.iterrows():
    # print(row['Frame#'], row["secs"], row["nsecs"],  row['x'], row['y'], row['z'],row['l'], row['w'], row['h'],row['orientation'], row['confidence_level'])
    print(
    "Frame#:", row['Frame#'], type(row['Frame#']),
    "secs:", row["secs"], type(row["secs"]),
    "nsecs:", row["nsecs"], type(row["nsecs"]),
    "x:", row['x'], type(row['x']),
    "y:", row['y'], type(row['y']),
    "z:", row['z'], type(row['z']),
    "l:", row['l'], type(row['l']),
    "w:", row['w'], type(row['w']),
    "h:", row['h'], type(row['h']),
    "orientation:", row['orientation'], type(row['orientation']),
    "confidence_level:", row['confidence_level'], type(row['confidence_level'])
)
    
    if index == 10:
        break
    
