clc;
clear;
close all;

global beta; global Tau; global h;

% Define some simulation parameters
beta = 0.02; Tau = 0.5; h = 0.01; reach_dist = 0.010; rot_dist = 0.02;
jointspace_dist = 0.002; % error tolerance for jointspace goal
alpha = 0.02;            % scales the error vector in joint space

% Define initial and final joint and end-effector configurations
% Initial
% theta_initial = [0.0947; -1.3848; -0.3447; 2.2595; 0.1863; -0.8544; -0.0747];
theta_initial = [-0.303;-0.762;-0.246;1.168;0.095;0.899;0.120];
[g_initial, ~, ~, ~, g_intermediate_initial] = FK_URDF(theta_initial);
A = Mat2DQ(g_initial);
q_o = theta_initial;

% Final
% theta_final = [-1.3061; -0.5065; 0.4091; 0.9871; -0.4022; -0.4525; 0.0855];
theta_final = [-0.459245;-0.342884;-0.906665;1.0694;0.939092;1.14118;-0.303671];
[g_final, ~, ~, ~, g_intermediate_final] = FK_URDF(theta_final);
B = Mat2DQ(g_final);

% Visualize initial and goal configurations
hfig = figure(1);

% Initial arm configuration
arm_cylinder_model(g_intermediate_initial);
plot_motion(hfig);

% Goal arm configuration 
arm_cylinder_model(g_intermediate_final);
plot_motion(hfig);

% Main loop of iterative motion plan using PATH solver
count = 1; currentDQ = A; finalDQ = B;
[g, ~, ~, ~, g_intermediate] = FK_URDF(q_o);
not_reached_flag = true; 
not_achieved_jointspace_goal = true;
theta_SCLERP = []; 
theta_SCLERP(:, count) = q_o;
plot_motion(hfig);

while not_reached_flag || not_achieved_jointspace_goal
    % Compute g_tau and DQ tau 
    [~, ResDQ] = Screw_Lin(currentDQ, finalDQ, Tau);
    
    % Move end effector to g_tau by approximating joint angles
    J_st = Jac_URDF(q_o);
    [thetha_next, v_ts2js] = theta_next_robust_config(J_st, g, ResDQ, q_o, theta_final, beta, alpha);
    q_o = thetha_next;
    
    % Check if q_o is within joint limits
    if ~check_jlimits(thetha_next)
        fprintf("Joint limit violated. \n");
        break;
    end
    
    [g, ~, ~, ~, g_intermediate] = FK_URDF(thetha_next);
    currentDQ = Mat2DQ(g);
    count = count+1;
    theta_SCLERP(:, count) = thetha_next;
    
    % Check how far from goal
    [rotation_err, position_err] = distDQ(ResDQ, currentDQ);
    jointpsace_err = norm(theta_final - q_o); 
    
    if position_err < reach_dist && rotation_err < rot_dist
        not_reached_flag = false;
        fprintf("taskspace goal reached according to given tolerance. \n");
    end
    
    if jointpsace_err < jointspace_dist
        not_achieved_jointspace_goal = false;
        fprintf("jointspace goal reached according to given tolerance. \n");
    end
    
    fprintf("iteration=%d\n",count);
    fprintf("pos err: %2.6f, rot err: %2.6f\n", position_err, rotation_err);
    fprintf("jointspace err: %2.6f\n", jointpsace_err);
    fprintf("----------\n");
    
    %/////////////// Plot the motion ///////////////
    arm_cylinder_model(g_intermediate);
    plot_motion(hfig);
end

save("joint_angles.mat", "theta_SCLERP");

% Write the path in a text file
fileID = fopen('joint_path.txt', 'w');
fprintf(fileID,'%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f,%2.4f\n',theta_SCLERP);
fclose(fileID);
