# CarND-Path-Planning-Project

## Introduction
The purpose of the Path Planning Project is to solve the problem of self-driving car in a virtual highway by smoothing it. This leads us to answer the following issues :

1. Smoothing the existing waypoints
2. Generating minimum jerk trajectories based on a minimal set of controls
3. Determining the best action based on model cost analysis
4. Transforming the generated trajectory from Frenets coordinates to cartensian ones

## Analysis and Design
The Analysis of the project's given data leads to different logical components :
1. Vehicle : Responsible of keeping track of its current lane, its state, neighbor vehicles, front and back gap
2. Lane : Reponsible of providing information on the current lane like distance and neighboor lanes
3. Behavior Planner : Responsible of computing gap, cost for the ego vehicle and finaly decide of the lane changing
4. Trajectory : Responsible of velocity limitation, and data preparation for JMT computing
5. Jerk Minimizing Trajectory : Responsible of smoothing the trajectory by constraining the calculation process to the minimum jerk 
6. Path Transform : Responsible of transforming the generated path in frenet coordinates to cartesian coordiantes
7. Main : Responsible of handling requests of way points from the simulator, calculates the ego vehicle environment, the ego vehicle trajectory by minimizing the jerk and sends back the converted the trajectory in cartesian coordinates to the simulator 
