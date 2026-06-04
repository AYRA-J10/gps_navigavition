 # GPS-Denied Navigation System

1st year sem break project (personal)

# What it does
Vehicle navigation without GPS using:
- IMU (drifts over time)
- Magnetic field matching (no drift, but ambiguous)
- Kalman filter (fuses both)

# Results (10x10 map, 50 sec simulation)
- IMU alone error: 2.17 cells
- Magnetic alone error: 10.82 cells
- Kalman fusion error: 2.29 cells (78% improvement)

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
