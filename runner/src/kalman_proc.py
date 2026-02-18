import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.stats import norm
import csv
import pandas as pd


def get_column_from_csv(csv_file_path, column_headers):
    df = pd.read_csv(csv_file_path)
    
    if isinstance(column_headers, str):
        column_headers = [column_headers]
    
    col = df[column_headers].to_numpy()
    return col

def plot_data_series(data_arrays, labels, x_label="Index", y_label="Value", title="Plot", markers=False):

    if len(data_arrays) != len(labels):
        raise ValueError("The number of data arrays must match the number of labels.")
    
    plt.figure(figsize=(10, 6))
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)

    for array, label in zip(data_arrays, labels):
        if markers:
            plt.plot(array, marker='o', label=label)
        else:
            plt.plot(array, label=label)

    plt.legend()
    plt.show()

def gyro_to_angle(gyro_readings, d_t):
    total_angle = 0.0
    result = []
    for reading in gyro_readings:
        total_angle += reading * d_t * (180/math.pi)
        result.append(total_angle)  
    return result

def accel_to_pitch(ax_list, ay_list, az_list):
    pitches = []
    for (ax, ay, az) in zip(ax_list, ay_list, az_list):
        sqrt = math.sqrt(ax*ax + az*az)
        div = ay/sqrt
        pitches.append((ay,sqrt,div))
    return pitches

def arctan_calculator_normal_angles(num_samples=1000, mean_deg=0.0, std_deg=40.0):
    angles_deg = np.random.normal(loc=mean_deg, scale=std_deg, size=num_samples)
    angles_deg = np.clip(angles_deg, 45, 89)
    angles_rad = np.sort(np.deg2rad(angles_deg))
    y_values = np.tan(angles_rad)
    
    res = [(math.atan(y), y) for y in y_values]
    return res

def arctain_calculator_uniform(min=0,max=45,step=0.05):

    angles = np.arange(min, max + step, step)  # in degrees
    angles_rad = np.radians(angles).clip(min=0,max=1.55334)
    y_values = np.tan(angles_rad)

    res = [(math.atan(y), y) for y in y_values]

    return res

def float_to_fixed_int(x: float,scale = 13) -> str:
    return int(x*(2**scale))

def display_graph(merged_date):
    computed_angles_deg = [math.degrees(angle) for angle, _ in merged_data]
    
    plt.figure(figsize=(8, 6))
    count, bins, ignored = plt.hist(computed_angles_deg, bins=50, density=True,
                                    alpha=0.6, color='g', label="Data Histogram")
    

    x = np.linspace(-89, 89, 2000)
    pdf = norm.pdf(x, loc=0, scale=15)
    plt.plot(x, pdf, 'k', linewidth=2, label="Expected Normal Curve")
    
    plt.xlabel("Angle (degrees)")
    plt.ylabel("Probability Density")
    plt.title("Bell Curve of Generated Angles")
    plt.legend()
    plt.show()

def rust_code(y,angle):
    y = list(y)
    y.sort()
    angle = list(angle)
    angle.sort()
    values = list(zip(angle,y))

    list_length = len(values)
    file_path = "runner/src/lut_code.txt"

    with open(file_path, 'w') as file:
        file.write("\tconst LUT:[(u8,u16);{}] = [\n".format(list_length))
        for angle, y in values:
            file.write("\t\t({},{}),\n".format(angle,y))
        file.write("\t];")


if __name__ == "__main__":
    angle_list = []
    angle = 0
    file_path = "drone_logging_data_2025-03-31_15-01-08.csv"
    headers = ["gyro_z"]

    data = get_column_from_csv(file_path,headers)
    sum = 0
    for i in range(0,200):
        sum = sum + data[i][0]

    offset = sum/200

    max = 0
    for reading in data:
        red = reading[0]/16.4
        if red > max:
            max = red
    
    print("Largest value is {}".format(max))
        

    for z_reading in data:
        angle = (angle + ( ((z_reading[0])-offset)/16.4)  * 0.004)
        angle_list.append(angle)

    print(offset)
    plot_data_series([angle_list], headers, markers=False)

    # # Generate data (list of tuples)
    # data_uniform = arctain_calculator_uniform(max=89,step=0.05)
    # # data_normal = arctan_calculator_normal_angles(num_samples=1000, mean_deg=45, std_deg=47.0)
    # merged_data = data_uniform
    # merged_data.sort(key=lambda t:t[0])

    # y_set = set()
    # angle_set = set()

    # with open("fixed_data.csv", "w", newline="") as csvfile:
    #     writer = csv.writer(csvfile)
    #     # Write a header row.
    #     writer.writerow(["bin_angle_rad_fixed","angle_rad","bin_y_fixed","y"])

    #     # Process each (angle, y) tuple.
    #     for angle, y in merged_data:
    #         # Convert each float to its 16-bit fixed-point binary representation.
    #         angle_bin = float_to_fixed_int(angle,7)
    #         y_bin = float_to_fixed_int(y,7)
    #         if not angle_bin in angle_set and not y_bin in y_set:
    #             writer.writerow([angle_bin,angle,y_bin,y])
    #             y_set.add(y_bin)
    #             angle_set.add(angle_bin)
    
    # print("Data written to arctan_data.csv")
    
    # #display_graph(merged_data)

    # rust_code(y_set,angle_set)



