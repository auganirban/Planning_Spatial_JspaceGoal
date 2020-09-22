clc;
clear;
close all;
clearvars -global;

% Add path to PATH solver
addpath("../supporting_quatlib"); addpath("../supporting_colli_funcs");
addpath("../supporting_kinlib");  addpath("../pathmexa64"); addpath("../supporting_colli_models");
addpath("../supporting_visualize");

global Tau; global beta; global h; global safe_dist; global task_dimen;
global dof; global hfig;

hfig  = figure(1); hold on; view(164, 47);
title("Baxter left arm transferring object through the window"); xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

%////////////////////// Inputs/////////////////////////
theta0 = [-0.2866, -0.8894, -0.3209, 2.0856, 0.6907, -1.1725, 0.5191]';
window_position = [0.850; 0.250; 0.300]; % Joint positions of the left arm of Baxter robot
% [~, box_vertices_array, box_normals_array] = get_obstacle_wall(window_position);

% Planning algorithm parameters
beta = 0.02; Tau = 0.5; h = 0.010; reach_dist = 0.005; rot_dist = 0.02;
dof = 7; safe_dist = 0.010; num_of_contacts = 7; count = 1; task_dimen = 6;

% Initial loaction in task space
g = FK_URDF(theta0); A = Mat2DQ(g);

g2 = [g(1:3, 1:3), [window_position(1)+0.050; window_position(2); window_position(3)]; zeros(1, 3), 1];
plot3(g2(1, 4), g2(2, 4), g2(3, 4), 'b*');
B = Mat2DQ(g2);

% Move from initial to goal pose
[actual_reached_goal] = get2goal(theta0, A, B);

function[achieved_goalDQ] = get2goal(theta0, A, B)
    % Define global variables
    global g;              global q_o;         global v_ts2js;
    global dof;            global dista;       global count;
    global contact_normal; global jcon_array;  global g_intermediate; 
    global J_st;           global Tau;         global beta;
    global box_vertices_array; global box_normals_array; global hfig;
    
    count = 1; reach_dist = 0.010; rot_dist = 0.02;
    prev_position_err = 100; prev_rotation_err = 100;
    
    % Set the bounds of unknowns for PATH Solver
    for j = 1 : 14
        l(j) = -Inf; u(j) = Inf;
    end
    l(1, 8:14) = 0;   % sice complementarity velocity is always >= 0 
    z = zeros(14, 1); % first 7 are joint_q and last 7 are compensating velocities
    q_o = theta0;     % initial config

    % Main loop of iterative motion plan using PATH solver
    currentDQ = A;
    finalDQ = B;
    [g, ~, ~, ~, g_intermediate] = FK_RealBaxter(q_o);
    not_reached_flag = true;
    
    while not_reached_flag
        % Compute g_tau and DQ tau    
        [~, ResDQ] = Screw_Lin(currentDQ, finalDQ, Tau);
        % Move end effector to g_tau by approximating joint angles   
        J_st = JS_Re(q_o);
        [thetha_next, v_ts2js] = theta_next_step2(J_st, g, ResDQ, q_o, beta);
%         [contact_normal, dista, jcon_array] = get_collision_info4(g_intermediate, box_vertices_array, box_normals_array);
        [contact_normal, dista, jcon_array] = get_collision_info4_graspobj(g_intermediate, box_vertices_array, box_normals_array);
        [z, ~, ~] = pathmcp(z, l, u, 'mcpfuncjacEval');
        q_o = z(1:dof);
        [g, ~, ~, ~, g_intermediate] = FK_URDF(q_o);
        currentDQ = Mat2DQ(g);
        count = count+1;
        theta_SCLERP(:, count) = q_o;

        % Check how far from goal
        [rotation_err, position_err] = distDQ(ResDQ, currentDQ);
        
        if (prev_position_err - position_err) > 1e-4 || (prev_rotation_err - rotation_err) > 1e-4
            prev_position_err = position_err;
            prev_rotation_err = rotation_err;
        else
            not_reached_flag = false;
            fprintf("position or rotation errors do not improve. \n");
        end

        if position_err < reach_dist && rotation_err < rot_dist
            not_reached_flag = false;
            fprintf("goal reached according to given tolerance. \n");
        end

        %/////////////// Plot the motion ///////////////    
        % Plot motion
    % % %     plot_motion(hfig, myVideo);
        plot_motion(hfig);
    end
    achieved_goalDQ = currentDQ;
end