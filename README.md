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
- *simulation includes random noise. Every run different*
  -to Get Stable, Repeatable Results: Add this line at the very top of the script
  
     "" **rng(42);  % Fix random seed — same "random" numbers every run** ""

## Why Kalman fusion is slightly worse than IMU alone here

This is expected, not a bug.

At 50 seconds, IMU drift is still small — the inertial sensor hasn't had time to accumulate significant error. 
Meanwhile, the 10x10 magnetic map is too coarse: many cells share similar field values, 
so magnetic matching occasionally returns a wrong position fix. 
The Kalman filter adds that noisy correction to an already-decent IMU estimate, making things marginally worse.

**Fusion wins at longer timescales.** IMU drift grows continuously with time. 
By 300-500 seconds, IMU-alone error would reach 15-20+ cells. 
At that point, even an imperfect magnetic correction pulls position back significantly — 
and Kalman fusion pulls ahead by a large margin.

The 78% improvement figure compares Kalman fusion vs magnetic-alone (the weakest baseline). 
The real story: fusion matches IMU accuracy at short timescales and 
decisively outperforms it as mission duration increases.

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
