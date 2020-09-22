% /////////////////////////////////////////////////////////////////////////
%   Input: theta_initial, robustIK for theta_final                     ////
%   Output: path from theta_initial to theta_final, extra              ////
%   straight down path for grasping.                                   ////
%   Author: Anirban Sinha                                              ////
%   Affiliation: Stony Brook University                                ////
%   Date: July 22nd, 2020.                                             ////
%//////////////////////////////////////////////////////////////////////////

clc;
clear;
close all;
clearvars -global;

% Define global variables
global h;                  global Tau;                global beta;                
global count;              global dof;                global alpha;
global num_of_contacts;    global task_dimen;         global safe_dist;
global box_vertices_array; global box_normals_array;  global num_links_ignore;
global jointspace_dist;    global reach_dist;         global rot_dist;
global choose_visualize;   global num_obstacles;      global obstacle_box_data;
global myVideo;            global must_achieve_theta_goal;

% Specify number of links to ignore in collision checking
num_links_ignore =  5;

% % % % Make visualization true or false
choose_visualize = true;
% % % myVideo = VideoWriter("motion"); % open video file
% % % myVideo.FrameRate = 30;                % can adjust this, 5 - 10 works well for me
% % % open(myVideo);

% Make must_achieve_theta_goal true or false
must_achieve_theta_goal = true;

hfig  = figure(1); hold on; view(164, 47);
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

%////////////////////////////////////////////////////////////////////////
%///////////////////////// Input set ////////////////////////////////////
%////////////////////////////////////////////////////////////////////////
% Define Obstacle Frame Locations wrt Baxter base ([x,y,z])
obs_frame_location = [0.95,0.6,-0.2;
                      0.50,0.13,-0.2]';
num_obstacles = size(obs_frame_location,2);
obstacle_box_data = {load("obs_nrm_vrtx3.mat"), load("obs_nrm_vrtx4.mat")};
[~, box_vertices_array, box_normals_array] = load_obstacles(obs_frame_location);

% Planning algorithm parameters
beta = 0.02; alpha = 0.01; Tau = 0.5; h = 0.01; 
reach_dist = 0.0001; rot_dist = 0.002; jointspace_dist = 0.1;
safe_dist = 0.010; num_of_contacts = 7;  
task_dimen = 6; dof = 7; count = 1;
hov_dist = 0.050;

% Initial and final joint space
theta_initial = [0.308,-0.500,-0.786,0.802,0.252,0.392,0.116]';

% Robust IK
theta_final = [-0.453771;-0.00311949;-2.61473;0.940007;2.3854;2.05735;-0.0946908]; 

%////////////////////////////////////////////////////////////////////////
%//////////////////////////// Initialize ////////////////////////////////
%////////////////////////////////////////////////////////////////////////
[g_initial, ~, ~, ~, g_intermediate_initial] = FK_RealBaxter(theta_initial);
[g_final, ~, ~, ~, g_intermediate_final] = FK_RealBaxter(theta_final);

A = Mat2DQ(g_initial);
B = Mat2DQ(g_final);

q_o = theta_initial;

%////////////////////////////////////////////////////////////////////////
%//////////////////////////// Compute the approach plan /////////////////
%////////////////////////////////////////////////////////////////////////
[theta_SCLERP, achieved_g2, achieved_DQ2, achieved_jointvec2] = ...
    get2goalRobustIK(theta_initial, A, B, theta_final);
% % % close(myVideo);

%////////////////////////////////////////////////////////////////////////
%/////////////////////////// Write the path in a file ///////////////////
%////////////////////////////////////////////////////////////////////////
save("joint_angles_ex.mat", "theta_SCLERP");
fileID = fopen('joint_path_obs_ex.txt', 'w');
fprintf(fileID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',theta_SCLERP);
fclose(fileID);

%/////////////////////////////////////////////////////////////////////////
%/////////////////// Straight down motion ////////////////////////////////
%/////////////////////////////////////////////////////////////////////////
straight_dwn_path = straight_down_motion(theta_SCLERP(:,end), hov_dist);

save("straight_dwn_path.mat", "straight_dwn_path");
fileID2 = fopen('straight_dwn_path.txt', 'w');
fprintf(fileID2,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',straight_dwn_path);
fclose(fileID2);

% % % theta_initial = [0.442;-0.489;-1.499;1.031;1.186;1.275;-0.223];
% % % theta_initial = [0.1053771;-0.00311949;-2.61473;0.940007;2.3854;2.05735;-0.0946908];
% % % [g_initial, ~, ~, ~, g_intermediate_initial] = FK_URDF(theta_initial);