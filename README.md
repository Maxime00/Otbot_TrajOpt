# Otbot_TrajOpt
Trajectory optimization for omnidirectional tire-wheeled robot. Complete system dynamics are available, with a wrapper to model any constraints into the optimization scheme.

# Otbot-models
The complete kinematic and dynamic models for the Otbot can be found in the folder Otbot-models.
The Results and model folder must be added to path before running.

#Otbot-TrajOpt
A wrapper of the OptimTraj library for the Otbot can be found in the OtbotTrajOpt/Otbot folder. 
It includes everything necessary to create any desired scenario and comptue optimal trajectories using direct collocation as implemented in OptimTraj.

## Main
To compute a trajectory, simply run the MAIN_loop.m with your desired parameters. 
This fucntion is designed to run several optimizations in a row, for each set of parameters provided.
A list of possibe plots is also available.

## Obstacles
To model obstacles, use the files in the Obstacles folder. The number of obstacles must be provided accurately for the optimization to run.
 
## Videos
To make videos of simulation, use the RESULTS_makeplots.m function. Simply provide the name of your solution and put 'yes' for the make_video variable.
A list of possibe plots is also available.

#IMPORTANT
current folder must be Otbot-TrajOpt when running to save files in correct emplacement. OptimTraj and OTbot folder must both be added to matlab path.
All results will be saved in the Otbot/Results folder.
