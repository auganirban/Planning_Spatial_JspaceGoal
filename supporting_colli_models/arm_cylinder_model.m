function [link_cylinder_array] = arm_cylinder_model(g_intermediate)
    global link1_cylinder1; global link2_cylinder2; global link3_cylinder3;
    global link4_cylinder4; global link5_cylinder5; global link6_cylinder6;
    global link7_cylinder7; global link8_cylinder8; global link9_cylinder9;
    global link10_cylinder10; global n_pts;
    
    % Required input: th (arm joint position vector of size 1X7).
    %
    % This code models the Baxter left arm links as cylinders which will be
    % useful to consider collision checking steps. The robot model is consists
    % of 7 main links and 3 offset links. To check collision, we will only
    % consider the 7 main links as floating cylinders and ignore the offset
    % links.
    % 
    % Add custom libraries to the Matlab path
    % addpath("supporting_kinlib");
    %
    % Note1: Frames shown in the plot are based on axis of rotations.
    % However cylinder informations are returned as Z axis aligned with the
    % axis of the cylinder.
    %
    % Note2: Offset links are not considered for collision checks.
    %
    % Date: November 13th, 2019.
    %
    %////////////////////////////////////
    %      Solve forward kinematics    //
    %////////////////////////////////////
%     [g_current, ~, ~, ~, g_intermediate] = FK_URDF(th);

    % ///////////////////////////////
    n_pts = 20;
    link_cylinder_array = cell(1, 7);

    % /////////// Link 1 ////////////
    rc = 0.010;
    [X, Y, Z] = cylinder(rc, n_pts-1);
    hc1 = g_intermediate(3, 4, 2) - g_intermediate(3, 4, 1);
    Z1 = hc1 * Z;
    cylinder_pts1 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(Z1, [1, 2*n_pts])];
    link1_cylinder1 = g_intermediate(1:3, 1:3, 1) * cylinder_pts1 + g_intermediate(1:3, 4, 1);
    link1_info = link_cylinder_info(g_intermediate(:, :, 1), hc1, rc, g_intermediate(1:3, 4, 1));
    link_cylinder_array{1, 1} = link1_info;

    % /////////////// Offset 1 ////////////
    Z2_dir = g_intermediate(1:3, 4, 2) - (g_intermediate(1:3, 4, 1) + [0; 0; hc1]);
    hc2 = norm(Z2_dir);
    Z_ax = Z2_dir/hc2;
    X_ax = -g_intermediate(1:3, 3);
    Y_ax = cross(Z_ax, X_ax);
    cylinder_pts2 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc2 * Z, [1, 2*n_pts])];
    link2_cylinder2 = [X_ax, Y_ax, Z_ax] * cylinder_pts2 + (g_intermediate(1:3, 4, 1) + [0; 0; hc1]);

    % //////////////// Link 2 ///////////////
    rc3 = 0.010;
    hc3 = norm(g_intermediate(1:3, 4, 3) - g_intermediate(1:3, 4, 2));
    [X, Y, Z] = cylinder(rc3, n_pts-1);
    cylinder_pts3 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc3 * Z, [1, 2*n_pts])];
    link3_cylinder3 = g_intermediate(1:3, 1:3, 3) * cylinder_pts3 + g_intermediate(1:3, 4, 2);
    cylinder_transform2 = [g_intermediate(1:3, 1:3, 3), g_intermediate(1:3, 4, 2); zeros(1, 3), 1];
    link2_info = link_cylinder_info(cylinder_transform2, hc3, rc3, g_intermediate(1:3, 4, 2));
    link_cylinder_array{1, 2} = link2_info;

    % /////////////// Link 3 /////////////////
    rc4 = 0.010;
    hc4 = sqrt((norm(g_intermediate(1:3, 4, 4) - g_intermediate(1:3, 4, 3)))^2 - (0.069)^2);
    [X, Y, Z] = cylinder(rc4, n_pts-1);
    cylinder_pts4 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc4 * Z, [1, 2*n_pts])];
    link4_cylinder4 = g_intermediate(1:3, 1:3, 3) * cylinder_pts4 + g_intermediate(1:3, 4, 3);
    link3_info = link_cylinder_info(g_intermediate(:, :, 3), hc4, rc4, g_intermediate(1:3, 4, 3));
    link_cylinder_array{1, 3} = link3_info;

    % //////////////// Offset 2 //////////////////
    rc5 = 0.010;
    hc5 = 0.069;
    [X, Y, Z] = cylinder(rc5, n_pts-1);
    cylinder_pts5 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc5 * Z, [1, 2*n_pts])];
    temp_pt = g_intermediate(1:3, 4, 3) + hc4 * g_intermediate(1:3, 3, 3);
    Z_ax = temp_pt - g_intermediate(1:3, 4, 4); Z_ax = Z_ax / norm(Z_ax);
    X_ax = g_intermediate(1:3, 3, 4);
    Y_ax = cross(Z_ax, X_ax);
    link5_cylinder5 = [X_ax, Y_ax, Z_ax] * cylinder_pts5 + g_intermediate(1:3, 4, 4);

    % //////////////////// Link 4 ///////////////////
    rc6 = 0.010;
    hc6 = norm(g_intermediate(1:3, 4, 4) - g_intermediate(1:3, 4, 5));
    [X, Y, Z] = cylinder(rc6, n_pts-1);
    cylinder_pts6 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc6 * Z, [1, 2*n_pts])];
    link6_cylinder6 = g_intermediate(1:3, 1:3, 5) * cylinder_pts6 + g_intermediate(1:3, 4, 4);
    cylinder_transform4 = [g_intermediate(1:3, 1:3, 5), g_intermediate(1:3, 4, 4); zeros(1, 3), 1];
    link4_info = link_cylinder_info(cylinder_transform4, hc6, rc6, g_intermediate(1:3, 4, 4));
    link_cylinder_array{1, 4} = link4_info;

    % //////////////////// Link 5 ////////////////////
    hc7 = sqrt((norm(g_intermediate(1:3, 4, 6) - g_intermediate(1:3, 4, 5)))^2 - (0.010)^2);
    rc7 = 0.010;
    [X, Y, Z] = cylinder(rc7, n_pts-1);
    cylinder_pts7 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc7 * Z, [1, 2*n_pts])];
    link7_cylinder7 = g_intermediate(1:3, 1:3, 5) * cylinder_pts7 + g_intermediate(1:3, 4, 5);
    link5_info = link_cylinder_info(g_intermediate(:, :, 5), hc7, rc7, g_intermediate(1:3, 4, 5));
    link_cylinder_array{1, 5} = link5_info;

    % ////////////////////// Offset 3 ///////////////////
    hc8 = 0.010;
    rc8 = 0.010;
    [X, Y, Z] = cylinder(rc8, n_pts-1);
    cylinder_pts8 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc8 * Z, [1, 2*n_pts])];
    temp_pt = g_intermediate(1:3, 4, 5) + hc7 * g_intermediate(1:3, 3, 5);
    Z_ax = temp_pt - g_intermediate(1:3, 4, 6); Z_ax = Z_ax / norm(Z_ax);
    X_ax = g_intermediate(1:3, 3, 6);
    Y_ax = cross(Z_ax, X_ax); Y_ax = Y_ax/norm(Y_ax);
    link8_cylinder8 = [X_ax, Y_ax, Z_ax] * cylinder_pts8 + g_intermediate(1:3, 4, 6);

    % //////////////////////// Link 6 //////////////////////
    hc9 = norm(g_intermediate(1:3, 4, 7) - g_intermediate(1:3, 4, 6));
    rc9 = 0.010;
    [X, Y, Z] = cylinder(rc9, n_pts-1);
    cylinder_pts9 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc9 * Z, [1, 2*n_pts])];
    link9_cylinder9 = g_intermediate(1:3, 1:3, 7) * cylinder_pts9 + g_intermediate(1:3, 4, 6);
    cylinder_transform6 = [g_intermediate(1:3, 1:3, 7), g_intermediate(1:3, 4, 6); zeros(1, 3), 1];
    link6_info = link_cylinder_info(cylinder_transform6, hc9, rc9, g_intermediate(1:3, 4, 6));
    link_cylinder_array{1, 6} = link6_info;

    % /////////////////////// Link 7 ///////////////////////
    rc10 = 0.010;
    hc10 = norm(g_intermediate(1:3, 4, 8) - g_intermediate(1:3, 4, 7));
    [X, Y, Z] = cylinder(rc10, n_pts-1);
    cylinder_pts10 = [reshape(X, [1, 2*n_pts]); reshape(Y, [1, 2*n_pts]); reshape(hc10 * Z, [1, 2*n_pts])];
    link10_cylinder10 = g_intermediate(1:3, 1:3, 7) * cylinder_pts10 + g_intermediate(1:3, 4, 7);
    link7_info = link_cylinder_info(g_intermediate(:, :, 7), hc10, rc10, g_intermediate(1:3, 4, 7));
    link_cylinder_array{1, 7} = link7_info;
end

% %% Draw frame
% function draw_frame(R, p)
%     quiver3(p(1), p(2), p(3), R(1, 1), R(2, 1), R(3, 1), 0.025, 'r', 'LineWidth', 2.0);
%     quiver3(p(1), p(2), p(3), R(1, 2), R(2, 2), R(3, 2), 0.025, 'g', 'LineWidth', 2.0);
%     quiver3(p(1), p(2), p(3), R(1, 3), R(2, 3), R(3, 3), 0.025, 'b', 'LineWidth', 2.0);
% end