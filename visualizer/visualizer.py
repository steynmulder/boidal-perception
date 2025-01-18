import ast

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import ast

df = pd.read_csv('data.csv', sep=';')
# Strip leading/trailing spaces from column names
df.columns = df.columns.str.strip()

# Safely evaluate the 'Data' column as Python objects
df['Data'] = df['Data'].apply(ast.literal_eval)

# Function to compute differences for a given row
def compute_differences(row):
    x_diff = []
    y_diff = []
    for sublist in row:
        x_diff.append(sublist[0])
        y_diff.append(sublist[1])

    return x_diff, y_diff, row[0][0], row[0][1]

# Apply the function and split into two columns
df['X_Differences'], df['Y_Differences'], df['Trace_X'], df['Trace_Y'] = zip(*df['Data'].apply(compute_differences))

# Flatten the differences for plotting
timestamps = df['Timestamp']
x_differences = np.array(df['X_Differences'].to_list()).flatten()
y_differences = np.array(df['Y_Differences'].to_list()).flatten()
trace_x = np.array(df['Trace_X'].to_list())
trace_y = np.array(df['Trace_Y'].to_list())


# Duplicate timestamps to match the flattened differences
timestamps_repeated = np.repeat(timestamps, df['X_Differences'][0].__len__())

trace_x_repeated = np.repeat(df['Trace_X'], df['X_Differences'][0].__len__())
trace_y_repeated = np.repeat(df['Trace_Y'], df['Y_Differences'][0].__len__())

# Create the scatter plot
f, (ax1, ax2) = plt.subplots(1, 2, sharex=True, sharey=True)

f.set_figheight(9)
f.set_figwidth(16)

# Plot XY differences
ax1.scatter(timestamps_repeated, x_differences, color='blue', label='Error in x-position', s=1)
ax1.scatter(timestamps_repeated, trace_x_repeated, color='red', label='Trace x-position first boid', marker='^', s=0.1)

# Plot ZW differences
ax2.scatter(timestamps_repeated, y_differences, color='orange', label='Error in y-position', s=1)
ax2.scatter(timestamps_repeated, trace_y_repeated, color='red', label='Trace y-position first boid', marker='^', s=0.1)

# Add labels, legend, and title
ax1.set_xlabel('Timestamp')
ax2.set_xlabel('Timestamp')
ax1.set_ylabel('Differences')
ax2.set_title('Scatter Plot Grouped by Difference Type')
ax1.legend()
ax2.legend()
ax1.grid()
ax2.grid()
plt.show()