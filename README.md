# CarND-Path-Planning-Project

## Introduction
The purpose of the Path Planning Project is to solve the problem of self-driving car in a virtual highway by smoothing it. This leads us to answer the following requirements :

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

## Logical Architecture Description
### Vehicle : 
Each vehicle encapsulates some attributes and methods to manage its own information provided by the simulator
#### Attributes :
1. Identifier : makes the vehicle unique among the set of vehicles on the scene. This attributes is assigned automatically except for the ego vehicle which assigned manualy (id = 1000)
2. Left Gap, Right Gap and Current Gap : These attributes are of Gap type that memorizes the front and back gap from the ego vehicle to front and back (left, right, current lane's vehicle) vehicles. it is used for gap calculation by the Planner to make decision on lane change
3. S and D states : These attributes keep track of the s state(p=s,v=speed, a=0n-- we consider the velocity is constant) and the d state (p=d,v=0,a=0). The s and d states are used in the trajectory calculation
4. Lane : it represents the current lane. The lane attributes pointes to the logical component : lane and therefore can exploit its contained information

#### Methods : 
1. Update : 
