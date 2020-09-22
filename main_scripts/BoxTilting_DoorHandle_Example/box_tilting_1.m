clc;
clear;
close all;

% Define the sides of the box
a = 0.5; b = 0.25; c = 1.0; % lengths along X, Y, Z
p = 0.5*[a; b; c];

% Define position of the points A, B, C, E, F, G and H wrt point D.
p_D2A = [0; 0; c]; p_D2B = [a; 0; c]; p_D2C = [a; 0; 0];
p_D2E = [0; b; c]; p_D2F = [a; b; c]; p_D2G = [a; b; 0]; p_D2H = [0; b; 0];

% Define the frames
T_b2D_start = [rotm_x(pi/12)*rotm_y(-pi/12), zeros(3, 1); zeros(1, 3), 1];
T_D2CG = [eye(3, 3), p; zeros(1, 3), 1];
T_b2CG_start = T_b2D_start*T_D2CG;           % start pose

T_b2D_goal = [rotm_x(pi/5)*rotm_z(pi/5), zeros(3, 1); zeros(1, 3), 1];
T_b2CG_goal = T_b2D_goal*T_D2CG; % goal pose

% Convert start and goal poses into dual-quaternion format
startDQ = Mat2DQ(T_b2CG_start);
goalDQ = Mat2DQ(T_b2CG_goal);

% Interpolate between startDQ and goalDQ using ScLERP
figure(1);
h1 = fill3([-1, -1, 1, 1], [-1, 1, 1, -1], [0, 0, 0, 0], [0, 1, 0]);
xlabel("x"); ylabel("y"); zlabel("z");
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-0.5, 1.5]);
grid on;
hold on;
view(-81, -1);
alpha(h1, 0.6);

tau = 0:0.1:1;
count = 0;
for t = tau
    [T_b2CG, DQ_b2CG]= Screw_Lin(startDQ, goalDQ, t);
    T_b2D = T_b2CG*inv(T_D2CG);
    
    % Compute the vertices of the block
    R_b2D_inter = T_b2D(1:3, 1:3); p_b2D_inter = T_b2D(1:3, 4);
    p_b2A_inter = p_b2D_inter + R_b2D_inter * p_D2A;
    p_b2B_inter = p_b2D_inter + R_b2D_inter * p_D2B;
    p_b2C_inter = p_b2D_inter + R_b2D_inter * p_D2C;
    p_b2E_inter = p_b2D_inter + R_b2D_inter * p_D2E;
    p_b2F_inter = p_b2D_inter + R_b2D_inter * p_D2F;
    p_b2G_inter = p_b2D_inter + R_b2D_inter * p_D2G;
    p_b2H_inter = p_b2D_inter + R_b2D_inter * p_D2H;
    
    if mod(count, 3) == 0
        plot_tilted_box(p_b2A_inter, p_b2B_inter, p_b2C_inter, p_b2D_inter, p_b2E_inter, p_b2F_inter, p_b2G_inter, p_b2H_inter);
    
        for i = 1:3
            quiver3(T_b2D(1, 4), T_b2D(2, 4), T_b2D(3, 4), T_b2D(1, i), T_b2D(2, i), T_b2D(3, i), 0.5, "r");
            quiver3(T_b2CG(1, 4), T_b2CG(2, 4), T_b2CG(3, 4), T_b2CG(1, i), T_b2CG(2, i), T_b2CG(3, i), 0.5, "b"); 
        end
    end
    
    count = count + 1;
    
end

% % % % Setup plotting environment
% % % figure(1);
% % % xlabel("x"); ylabel("y"); zlabel("z");
% % % xlim([-1, 1]);
% % % ylim([-1, 1]);
% % % zlim([-0.5, 1.5]);
% % % grid on;
% % % hold on;
% % % view(31, 31);
% % % for i = 1:3
% % %     quiver3(T_b2D(1, 4), T_b2D(2, 4), T_b2D(3, 4), T_b2D(1, i), T_b2D(2, i), T_b2D(3, i), "r");
% % %     quiver3(T_b2CG(1, 4), T_b2CG(2, 4), T_b2CG(3, 4), T_b2CG(1, i), T_b2CG(2, i), T_b2CG(3, i), "b"); 
% % % end


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