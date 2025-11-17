# Thesis-Work-repository---KV3AM0
This is the official repository for the program files involved in my thesis work about UAV path planning and guidance.

This repository includes the following scripts required for running and related to the project:
1. **Path_finding.m** - This is the main MATLAB script, responsible for most calculations. See "How_to_run.txt" for instructions.
2. **plot-debug.m** - This script helps visually compare the planned paths and the simulation output by plotting them in the same 3D plot without the visual clutter of the 3D occupancy map. Good for checking suitability of the results.
3. **RRT_guidance.slx** - This SIMULINK model is responsible for calculating the path using the RRT algorithm
4. **BiRRT_guidance.slx** - This SIMULINK model is responsible for calculating the path using the BiRRT algorithm
5. **RRTstar_guidance.slx** - This SIMULINK model is responsible for calculating the path using the RRT* algorithm
6. **PRM_guidance.slx** - This SIMULINK model is responsible for calculating the path using the PRM algorithm
