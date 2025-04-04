import time
import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar

# ----- Configuration -----
# Update the port name for your system (e.g., 'COM3' on Windows)
PORT_NAME = 'COM4'
# Create an RPLidar instance; adjust baudrate and timeout as needed.
lidar = RPLidar(PORT_NAME, baudrate=115200, timeout=3)

# ----- Set Up Live Polar Plot -----
plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_title("RPLidar Mapping")
ax.set_rlim(0, 4000)

def update_plot(scan):
    """
    Update the polar plot using the latest scan data.
    Each measurement in 'scan' is a tuple: (quality, angle, distance)
    """
    angles = []
    distances = []
    for (_, angle, distance) in scan:
        if distance > 0:
            angles.append(np.deg2rad(angle))
            distances.append(distance)
    ax.clear()
    ax.set_title("RPLidar Mapping")
    ax.set_rlim(0, 4000)
    ax.scatter(angles, distances, s=2, c='blue')
    plt.pause(0.01)

# ----- Main Loop -----
try:
    print("Starting scan (press Ctrl+C to stop)...")
    # Increase the buffer limit to 1000 to reduce the frequency of buffer warnings
    for scan in lidar.iter_scans(max_buf_meas=1000):
        update_plot(scan)
except KeyboardInterrupt:
    print("Scan stopped by user.")
finally:
    lidar.stop()
    lidar.disconnect()
    print("Lidar connection closed.")