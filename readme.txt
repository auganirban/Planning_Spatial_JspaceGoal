1. This package is to compute motion plan of Baxter robot based on Taskspace based
sampling methods combined with complementarity constraint based contact modeling 
for obstacle avoidance.

2. Before using any of the scripts here, run the startup.m file at first so that
all the supporting function file paths are added to the MATLAB's search path.

3. All the main scripts (ex. randomized2goal_window_Jan4.m) can be found in 
"main_scripts" directory.

4. After the computation of the requested path, the output can be found in the
directory named "files_traj". (ex., plan_video_seed99.avi, data_seed98.mat, 
joint_vec_seed98.txt). The .avi file is corresponding to the video of the motion 
plan. The .txt file the useful to fed it to Baxter robot. The .mat file holds 
"tree" variable for further matlab analysis.

5. In any main scripts (ex. randomized2goal_window_Jan4.m), one must have to provide the following information,
(a) theta_initial (a 7X1 joint angle vector of initial configuration)
(b) theta_final (a 7X1 joint angle vector of final configuration)
(c) window_position (a 3X1 vector denoting center of the square window)
(d) window dimensions (window_h, window_w, window_t)

6. There are few other variables whose values can be tweaked to get better performance based on different situations. They are as follows,
(a) Planning algorithm parameters (beta, Tau, h, reach_dist, safe_dist)
(b) Parameters to control the bounding box of sample generation (box_h, box_w, box_d)
