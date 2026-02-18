import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter

# ----- Fixed-Point Helper Functions -----
def float_to_fixed(x, frac_bits):
    """Convert a float to fixed-point integer representation."""
    return int(round(x * (1 << frac_bits)))

def fixed_to_float(x_fixed, frac_bits):
    """Convert a fixed-point integer back to float."""
    return x_fixed / (1 << frac_bits)

def fixed_mul(a_fixed, b_fixed, frac_bits):
    """Multiply two fixed-point numbers, returning a fixed-point result."""
    return (a_fixed * b_fixed) >> frac_bits

# Parameters
fs = 250.0             # Sampling frequency (Hz)
f_c = 25.0             # Desired cutoff frequency (Hz)
order = 1              # <-- Use 2nd order (which yields 3 coefficients)
nyquist = fs / 2.0
Wn = f_c / nyquist     # Normalized cutoff frequency

# Design the Butterworth filter (2nd order => 3 coefficients)
b_float, a_float = butter(order, Wn, btype='low', analog=False)

# Print them just for reference
print("Floating-point b =", b_float)
print("Floating-point a =", a_float)

# Fixed-point config
FRAC_BITS = 6  # Q12.4 or similar
a_fixed = [float_to_fixed(val, FRAC_BITS) for val in a_float]
b_fixed = [float_to_fixed(val, FRAC_BITS) for val in b_float]

print("Fixed-point a =", a_fixed)
print("Fixed-point b =", b_fixed)

# Read CSV and Extract Data
csv_filename = "logging_data_2025-04-11_17-46-41.csv"
df = pd.read_csv(csv_filename)

accel_cols = ["accel_x", "accel_y", "accel_z"]
accel_data = df[accel_cols].to_numpy()

# Example: filter accel_y (column index 1)
accel = accel_data[:, 2]

# Convert input samples to fixed-point
accel_fixed = [float_to_fixed(x, FRAC_BITS) for x in accel]

# 2nd-order difference equation:
# y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
x_prev = [0, 0]  # x[n-1], x[n-2]
y_prev = [0, 0]  # y[n-1], y[n-2]

y_accel_fixed = []

for x_i in accel_fixed:
    # Compute y[n] in fixed point
    y_i = ( fixed_mul(b_fixed[0], x_i, FRAC_BITS)
          + fixed_mul(b_fixed[1], x_prev[0], FRAC_BITS)
        #   + fixed_mul(b_fixed[2], x_prev[1], FRAC_BITS)
          - fixed_mul(a_fixed[1], y_prev[0], FRAC_BITS)
        #   - fixed_mul(a_fixed[2], y_prev[1], FRAC_BITS) 
          )

    y_accel_fixed.append(y_i)
    
    # Update histories
    x_prev[1] = x_prev[0]
    x_prev[0] = x_i

    y_prev[1] = y_prev[0]
    y_prev[0] = y_i  # Store the newly computed sample here

# Convert fixed-point output back to float
y_accel = [fixed_to_float(val, FRAC_BITS) for val in y_accel_fixed]

# "Ideal" float filtering for comparison
ideal_y = lfilter(b_float, a_float, accel)

# Plot
plt.plot(accel, label="Unfiltered")
plt.plot(y_accel, label="Fixed Filtering")
plt.plot(ideal_y, label="Ideal Filtering")
plt.xlabel("Sampled at 0.004s",fontsize=14)
plt.ylabel("Filtered Accel",fontsize=14)
plt.tick_params(axis='both', labelsize=14)  # << increase tick sizes
plt.title("Filtered Accelerometer Data")
plt.legend(fontsize=11)
plt.show()
