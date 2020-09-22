function[joint_ang_vec] = straight_down_motion(theta_initial, hov_dist)
    hfig  = figure(); hold on; view(164, 47);
    xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

    % Planning algorithm parameters
    beta = 0.02; Tau = 0.5; 
    reach_dist = 0.001; 
    % hov_dist = 0.050;

    % Define initial joint configuration
    % % % theta_initial = [-0.3531,-0.9207,-0.7836,0.8860,0.4384,1.4567,0.0060]'; % natural-IK
    % % % theta_initial = [-0.4236,-0.0246,-2.5618,0.9366,2.3161,2.0374,-0.1182]'; % robust-IK
    [g_initial, ~] = FK_RealBaxter(theta_initial);
    A = Mat2DQ(g_initial);

    % Define goal pose of end-effector
    g_final = g_initial;
    g_final(3,4) = g_final(3,4) - hov_dist;
    R_g_final = g_final(1:3, 1:3);
    B = Mat2DQ(g_final);

    scatter3(g_initial(1, 4), g_initial(2, 4), g_initial(3, 4), 'r', 'filled');
    hold on;
    scatter3(g_final(1, 4), g_final(2, 4), g_final(3, 4), 'g', 'filled');
    xlim([0, 1.2]); ylim([-0.8, 0.9]); zlim([-0.1, 0.9]); view(258, 19);

    %%%%%%%%%%%%%%%%%%%% Initialize %%%%%%%%%%%%%%%%%%%
    count = 1; 
    theta_SCLERP(:, count) = theta_initial;

    %%%%%%%%%%%%%%%%%%%% Planning loop %%%%%%%%%%%%%%%%%%%
    % Initiate the tree
    initial_dist = norm(g_final(1:3, 4) - g_initial(1:3, 4));

    % Main loop of iterative motion plan using PATH solver
    tau2 = 0.5;
    g_prev = g_initial;
    DQ_current = A;
    th_current = theta_initial;
    not_reached_flag = true;
    joint_ang_vec = [th_current];

    while not_reached_flag
        %%%%%%% Move DQ_closest to DQ_final %%%%%%%%%%%%%
        [ResMat, ResDQ] = Screw_Lin(DQ_current, B, tau2);
        J_st = Jac_RealBaxter(th_current);
        [th_current, ~] = theta_next_step2(J_st, g_prev, ResDQ, th_current, beta);
        g_current = FK_RealBaxter(th_current);
        DQ_current = Mat2DQ(g_current);
        g_prev = g_current;

        %%%%%%%%%%%%%%%%%%% Store new joint angle vector %%%%%%%%%%%%%%
        joint_ang_vec = [joint_ang_vec, th_current];

        %%%%%%%%%%%%%%%%%%%% Compute error %%%%%%%%%%%%%%%%%%%%
       [rotational_err, translation_err] = distDQ(DQ_current, B);
       count = count + 1;

       %%%%%%%%%%%%% Plot end-effector frames %%%%%%%%%%%%%%%%%%%%%%%%
       if mod(count, 20) == 0 
           for i = 1:3
                if i == 1
                    col_str = "r";
                elseif i == 2
                    col_str = "g";
                else
                    col_str = "b";
                end
                quiver3(g_current(1, 4), g_current(2, 4), g_current(3, 4), g_current(1, i), g_current(2, i), g_current(3, i), 0.05, col_str, 'linewidth',3); 
           end
       end

       if translation_err < reach_dist
           not_reached_flag = false;
       end

       count = count + 1;
    end
    
end
