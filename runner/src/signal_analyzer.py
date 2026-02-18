import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, filtfilt

def load_data(filepath):
    """Load CSV data into a pandas DataFrame."""
    return pd.read_csv(filepath)

def fft_column(dataframe, column_name, sampling_rate=1.0):

    if column_name not in dataframe.columns:
        raise ValueError(f"Column '{column_name}' not found in the DataFrame.")
    
    signal = dataframe[column_name].values
    n = len(signal)
    fft_result = np.fft.fft(signal)
    freqs = np.fft.fftfreq(n, d=1/sampling_rate)

    positive_freqs = freqs[:n//2]
    magnitude = np.abs(fft_result)[:n//2]

    plt.figure(figsize=(10, 5))
    plt.plot(positive_freqs, magnitude)
    plt.title(f"FFT of {column_name}")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    return positive_freqs, magnitude

def plot_columns(dataframe, columns):
    if isinstance(columns, str):
        columns = [columns]

    for col in columns:
        if col not in dataframe.columns:
            raise ValueError(f"Column '{col}' not found in the DataFrame.")

    plt.figure(figsize=(12, 6))
    for col in columns:
        plt.plot(dataframe[col], label=col)

    plt.title("Sensor Readings")
    plt.xlabel("Sample")
    plt.ylabel("Value")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def butterworth_filter(data, cutoff, fs, order=1, btype='low'):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = ([0.24523728,0.24523728],[1,-0.50952545])
    data = data/255
    filtered_data = filtfilt(b, a, data)
    return filtered_data

def plot_column_from_multiple_files(filepaths, column_name, labels=None):
    """
    Plot the same column from multiple CSV files on the same graph.

    Parameters:
    - filepaths: List of CSV file paths.
    - column_name: Column name to extract and plot.
    - labels: Optional list of labels for each line.
    """
    plt.figure(figsize=(12, 6))
    
    for i, filepath in enumerate(filepaths):
        df = pd.read_csv(filepath)
        if column_name not in df.columns:
            raise ValueError(f"'{column_name}' not found in {filepath}")
        
        label = labels[i] if labels else f"File {i+1}"
        plt.plot(df[column_name].values, label=label, marker = 'o')
    
    plt.xlabel("Sampeled every 0.004s", fontsize=18)
    plt.ylabel(column_name)
    plt.tick_params(axis='both', labelsize=14)  # << increase tick sizes
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

files = [
    "U5.csv",
    "U6.csv",
    "U7.csv"
]

plot_column_from_multiple_files(files, "gyro_x", labels=["5 fractional", "6 fractional", "7 fractional"])

# df = load_data(file_1)
# data_1 = df['gyro_x']
# print(max(data))
# cutoff = 25.0
# fs = 250
# order = 1
# df['filtered_gyro_x'] = butterworth_filter(data, cutoff, fs, order, btype='low')
# plot_columns(df, ['gyro_x', 'filtered_gyro_x'])

