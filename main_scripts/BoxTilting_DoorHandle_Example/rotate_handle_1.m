clc;
clear;
close all;

% Define the sides of the box
a = 0.05; b = 0.025; c = 0.50; % lengths along X, Y, Z
p_tip = [0; b/2; c];

% Define position of the points A, B, C, E, F, G and H wrt point D.
p_O2A = [-a/2; 0; c]; p_O2B = [a/2; 0; c]; p_O2C = [a/2; 0; 0]; p_O2D = [-a/2; 0; 0];
p_O2E = [-a/2; b; c]; p_O2F = [a/2; b; c]; p_O2G = [a/2; b; 0]; p_O2H = [-a/2; b; 0];
T_O2Tip = [eye(3, 3), p_tip; zeros(1, 3), 1];

% Define the frames
% Start and goal pose of the Frame at the origin O
T_b2O_start = [eye(3, 3), zeros(3, 1); zeros(1, 3), 1];
T_b2O_goal = [rotm_y(3*pi/4), zeros(3, 1); zeros(1, 3), 1];

% Start and goal pose of the Tip frame
T_b2Tip_start = T_b2O_start * T_O2Tip;
T_b2Tip_goal = T_b2O_goal * T_O2Tip;

% Convert start and goal poses into dual-quaternion format
startDQ = Mat2DQ(T_b2Tip_start);
goalDQ = Mat2DQ(T_b2Tip_goal);

% Interpolate between startDQ and goalDQ using ScLERP
figure(1);
% % % h1 = fill3([-1, -1, 1, 1], [-1, 1, 1, -1], [0, 0, 0, 0], [0, 1, 0]);
% % % alpha(h1, 0.6);
xlabel("x"); ylabel("y"); zlabel("z");
% % % xlim([-0.5, 1]); ylim([-0.5, 1]); zlim([-0.75, 0.75]);
axis("equal"); grid on; hold on; view(-81, -1);

% Actual ScLERP (used by our method)
ScLERP_inter(startDQ, goalDQ, T_O2Tip, p_O2A, p_O2B, p_O2C, p_O2D, p_O2E, p_O2F, p_O2G, p_O2H);

% Decoupled interpolation of position and orientation (comparison-1)
Decoupled_ScLERP(T_b2Tip_start, T_b2Tip_goal);

% % % % Velocity controller with error minimization (comparison-2)
% % % Velocity_Controller_Interpolation(T_b2Tip_start, T_b2Tip_goal);

% Helper functions
function [m] = rotm_x(th_x)
    m = [1, 0, 0;
        0, cos(th_x), -sin(th_x);
        0, sin(th_x), cos(th_x)];
end

function [m] = rotm_y(th_y)
    m = [cos(th_y), 0, sin(th_y);
        0, 1, 0;
        -sin(th_y), 0, cos(th_y)];
end

function [m] = rotm_z(th_z)
    m = [cos(th_z), -sin(th_z), 0;
         sin(th_z), cos(th_z), 0;
         0, 0, 1];
end

function [] = plot_tilted_box(ptA, ptB, ptC, ptD, ptE, ptF, ptG, ptH)
    h2 = fill3([ptA(1), ptB(1), ptC(1), ptD(1)], [ptA(2), ptB(2), ptC(2), ptD(2)], [ptA(3), ptB(3), ptC(3), ptD(3)], [0.5, 0.1, 0.1]);
    h3 = fill3([ptE(1), ptF(1), ptG(1), ptH(1)], [ptE(2), ptF(2), ptG(2), ptH(2)], [ptE(3), ptF(3), ptG(3), ptH(3)], [0.5, 0.0, 0.1]);
    h4 = fill3([ptA(1), ptE(1), ptH(1), ptD(1)], [ptA(2), ptE(2), ptH(2), ptD(2)], [ptA(3), ptE(3), ptH(3), ptD(3)], [0.5, 0.0, 0.1]);
    h5 = fill3([ptB(1), ptF(1), ptG(1), ptC(1)], [ptB(2), ptF(2), ptG(2), ptC(2)], [ptB(3), ptF(3), ptG(3), ptC(3)], [0.5, 0.0, 0.1]);
    h6 = fill3([ptA(1), ptE(1), ptF(1), ptB(1)], [ptA(2), ptE(2), ptF(2), ptB(2)], [ptA(3), ptE(3), ptF(3), ptB(3)], [0.5, 0.0, 0.1]);
    h7 = fill3([ptD(1), ptH(1), ptG(1), ptC(1)], [ptD(2), ptH(2), ptG(2), ptC(2)], [ptD(3), ptH(3), ptG(3), ptC(3)], [0.5, 0.0, 0.1]);
    
    alpha(h2, 0.1); alpha(h3, 0.1); alpha(h4, 0.1);
    alpha(h5, 0.1); alpha(h6, 0.1); alpha(h7, 0.1);
end

function [] = Decoupled_ScLERP(T_start, T_goal)
    count = 0;
    tau = 0:0.05:1;
    line_array = [];
    
    p_start = T_start(1:3, 4);
    p_goal = T_goal(1:3, 4);
    pos_dist = norm(p_start - p_goal);
    pos_vec = (p_goal - p_start)/pos_dist;
    T_b2Tip_inter = T_start;
    goalDQ = Mat2DQ(T_goal);
    for t = tau
        p_new = p_start + t*pos_dist*pos_vec;
        T_temp = [T_b2Tip_inter(1:3, 1:3), T_goal(1:3, 4); zeros(1, 3), 1];
        tempDQ = Mat2DQ(T_temp);
        [T_temp, ~]= Screw_Lin(tempDQ, goalDQ, t);
        T_b2Tip_inter = [T_temp(1:3, 1:3), p_new; zeros(1, 3), 1];
       
         if mod(count, 2) == 0
            for i = 1:3
                if i == 1
                    col_str = "r";
                elseif i == 2
                    col_str = "g";
                else
                    col_str = "b";
                end
                quiver3(T_b2Tip_inter(1, 4), T_b2Tip_inter(2, 4), T_b2Tip_inter(3, 4), T_b2Tip_inter(1, i), T_b2Tip_inter(2, i), T_b2Tip_inter(3, i), 0.05, col_str, 'linewidth',3); 
            end
        end
        line_array = [line_array, T_b2Tip_inter(1:3, 4)];

        count = count + 1;

    end
    plot3(line_array(1, :), line_array(2, :), line_array(3, :), 'm');
end

function [] = ScLERP_inter(startDQ, goalDQ, T_O2Tip, p_O2A, p_O2B, p_O2C, p_O2D, p_O2E, p_O2F, p_O2G, p_O2H)
    tau = 0:0.05:1;
    count = 0;
    
    line_array = [];
    for t = tau
        [T_b2Tip_inter, DQ_b2Tip_inter]= Screw_Lin(startDQ, goalDQ, t);
        T_b2O_inter = T_b2Tip_inter*inv(T_O2Tip);

        % Compute the vertices of the block
        R_b2O_inter = T_b2O_inter(1:3, 1:3); p_b2O_inter = T_b2O_inter(1:3, 4);

        p_b2A_inter = p_b2O_inter + R_b2O_inter * p_O2A;
        p_b2B_inter = p_b2O_inter + R_b2O_inter * p_O2B;
        p_b2C_inter = p_b2O_inter + R_b2O_inter * p_O2C;
        p_b2D_inter = p_b2O_inter + R_b2O_inter * p_O2D;
        p_b2E_inter = p_b2O_inter + R_b2O_inter * p_O2E;
        p_b2F_inter = p_b2O_inter + R_b2O_inter * p_O2F;
        p_b2G_inter = p_b2O_inter + R_b2O_inter * p_O2G;
        p_b2H_inter = p_b2O_inter + R_b2O_inter * p_O2H;

        if mod(count, 2) == 0
% % %             plot_tilted_box(p_b2A_inter, p_b2B_inter, p_b2C_inter, p_b2D_inter, p_b2E_inter, p_b2F_inter, p_b2G_inter, p_b2H_inter);

            for i = 1:3
                % quiver3(T_b2O_inter(1, 4), T_b2O_inter(2, 4), T_b2O_inter(3, 4), T_b2O_inter(1, i), T_b2O_inter(2, i), T_b2O_inter(3, i), 0.25, "r");
                if i == 1
                    col_str = "r";
                elseif i == 2
                    col_str = "g";
                else
                    col_str = "b";
                end
                quiver3(T_b2Tip_inter(1, 4), T_b2Tip_inter(2, 4), T_b2Tip_inter(3, 4), T_b2Tip_inter(1, i), T_b2Tip_inter(2, i), T_b2Tip_inter(3, i), 0.05, col_str, 'linewidth',3); 
            end
        end
        line_array = [line_array, T_b2Tip_inter(1:3, 4)];

        count = count + 1;

    end
    plot3(line_array(1, :), line_array(2, :), line_array(3, :), 'm');
end

function [] = Velocity_Controller_Interpolation(T_start, T_goal)
    tau = 0:0.05:1;
    line_array = [];
    count = 0;
    
    % parameters for position interpolation
    p_start = T_start(1:3, 4);
    p_goal = T_goal(1:3, 4);
    pos_dist = norm(p_start - p_goal);
    pos_vec = (p_goal - p_start)/pos_dist;
    
    % parameters for orientation interpolation
    q_start = rotm2quat(T_start(1:3, 1:3))';
    q_goal = rotm2quat(T_goal(1:3, 1:3))';
    quat_dist = norm(q_start - q_goal);
    quat_vec = (q_goal - q_start)/quat_dist;
    
    % Start linear interpolation
    for t = tau
        p_new = p_start + t*pos_dist*pos_vec;
        q_new = q_start + t*quat_dist*quat_vec;
        q_new = q_new/norm(q_new);
        
        T_b2Tip_inter = [quat2rotm(q_new'), p_new; zeros(1, 3), 1];
        
        if mod(count, 2) == 0
            for i = 1:3
                if i == 1
                    col_str = "r";
                elseif i == 2
                    col_str = "g";
                else
                    col_str = "b";
                end
                quiver3(T_b2Tip_inter(1, 4), T_b2Tip_inter(2, 4), T_b2Tip_inter(3, 4), T_b2Tip_inter(1, i), T_b2Tip_inter(2, i), T_b2Tip_inter(3, i), 0.05, col_str, 'linewidth',3); 
            end
        end
        
        line_array = [line_array, T_b2Tip_inter(1:3, 4)];
        count = count + 1;
    end
    plot3(line_array(1, :), line_array(2, :), line_array(3, :), 'm');
end

% % % position_start_DQ = Mat2DQ(T_position_start);
% % % position_goal_DQ = Mat2DQ(T_position_goal);
% % % 
% % % ScLERP_inter(position_start_DQ, position_goal_DQ, T_O2Tip, p_O2A, p_O2B, p_O2C, p_O2D, p_O2E, p_O2F, p_O2G, p_O2H);
% % % 
% % % % Interpolate orientation only
% % % T_rotation_start = [T_b2Tip_start(1:3, 1:3), T_b2Tip_goal(1:3, 4); zeros(1, 3), 1];
% % % T_rotation_goal = [T_b2Tip_goal(1:3, 1:3), T_b2Tip_goal(1:3, 4); zeros(1, 3), 1];
% % % 
% % % rotation_start_DQ = Mat2DQ(T_rotation_start);
% % % rotation_goal_DQ = Mat2DQ(T_rotation_goal);
% % % 
% % % ScLERP_inter(rotation_start_DQ, rotation_goal_DQ, T_O2Tip, p_O2A, p_O2B, p_O2C, p_O2D, p_O2E, p_O2F, p_O2G, p_O2H);