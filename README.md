 # GPS-Denied Navigation System

1st year sem break project (personal)

# What it does
Vehicle navigation without GPS using:
- IMU (drifts over time)
- Magnetic field matching (no drift, but ambiguous)
- Kalman filter (fuses both)

# Simulation Performance Analysis
By implementing a discrete-time Kalman Filter to fuse drifting IMU data with a noisy magnetic map, the algorithm stabilizes tracking and reduces positional drift by 97% over a 500-second timeline.
Validated Results:
• IMU Error: 96.89 cells
• Magnetic Error: 3.00 cells
• Kalman Fused Error: 2.86 cells

## Files
- `gps_denied_navigation.m` - Run this in MATLAB
- `magnetic_map.mat` - 10x10 magnetic map
- `finalgpsnavigation.png` - Screenshot of output

# How to run
1. Open MATLAB
2. Run `gps_denied_navigation.m`
3. View plots and errors

# Author
RIYA JOSEPH, B.Tech Mechatronics, 1st Year
