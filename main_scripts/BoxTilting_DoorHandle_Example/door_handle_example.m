clc;
clear;
close all;
clearvars -global;

% Define global variables
% global h;                  global Tau;         global beta;                global dof;
% global count;              global tree;        global grasped_box;         global not_reached_flag;
% global num_of_contacts;    global task_dimen;  global closest_sofar_index; global safe_dist;
% global box_vertices_array; global box_normals_array;

seed_val = 000000;
% % % rng(seed_val);
hfig  = figure(1); hold on; view(164, 47);
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

% Planning algorithm parameters
beta = 0.02; Tau = 0.5; h = 0.01; reach_dist = 0.010;
dof = 7; safe_dist = 0.010; num_of_contacts = 7; count = 1; task_dimen = 6;

% Define initial joint configuration
theta_initial = [0.0947, -1.3848, -0.3447, 2.2595, 0.1863, -0.8544, -0.0747]';
[g_initial, ~] = FK_RealBaxter(theta_initial);
A = Mat2DQ(g_initial);

% Define goal pose of end-effector
% g_final = [g_initial(1:3, 1:3), [window_position(1)+0.050; window_position(2); window_position(3)]; zeros(1, 3), 1];
theta_final = [-1.3061, -0.5065, 0.4091, 0.9871, -0.4022, -0.4525, 0.0855]';
[g_final, ~] = FK_RealBaxter(theta_final);
R_g_final = g_final(1:3, 1:3);
B = Mat2DQ(g_final);

scatter3(g_initial(1, 4), g_initial(2, 4), g_initial(3, 4), 'r', 'filled');
hold on;
scatter3(g_final(1, 4), g_final(2, 4), g_final(3, 4), 'g', 'filled');
xlim([0, 1.2]); ylim([-0.8, 0.9]); zlim([-0.1, 0.9]); view(258, 19);

%%%%%%%%%%%%%%%%%%%% Initialize %%%%%%%%%%%%%%%%%%%
count = 1; theta_SCLERP(:, count) = theta_initial;

%%%%%%%%%%%%%%%%%%%% Planning loop %%%%%%%%%%%%%%%%%%%
tic;
% Initiate the tree
initial_dist = norm(g_final(1:3, 4) - g_initial(1:3, 4));

% Main loop of iterative motion plan using PATH solver
tau2 = 0.2;
g_prev = g_initial;
DQ_current = A;
th_current = theta_initial;
not_reached_flag = true;
joint_ang_vec = [th_current];

while not_reached_flag  
    %%%%%%% Move DQ_closest to DQ_final %%%%%%%%%%%%%
    [ResMat, ResDQ] = Screw_Lin(DQ_current, B, tau2);
    J_st = JS_RealBaxter(th_current);
    [th_current, ~] = theta_next_step2(J_st, g_prev, ResDQ, th_current, beta);
    g_current = FK_RealBaxter(th_current);
    DQ_current = Mat2DQ(g_current);
    g_prev = g_current;
    
    %%%%%%%%%%%%%%%%%%% Store new joint angle vector %%%%%%%%%%%%%%
    joint_ang_vec = [joint_ang_vec, th_current];
    
    %%%%%%%%%%%%%%%%%%%% Compute error %%%%%%%%%%%%%%%%%%%%
   [rotational_err, translation_err] = distDQ(DQ_current, B);
   count = count + 1;

   if translation_err < 0.001
       not_reached_flag = false;
   end
end