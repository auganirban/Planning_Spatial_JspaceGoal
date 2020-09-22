function[theta_SCLERP, achieved_goal, achieved_goalDQ, achieved_jointvec] = get2goalRobustIK(theta0, A, B, theta_final)
    % Define global variables
    global g;                   global q_o;                global v_ts2js;
    global dof;                 global dista;              global count;
    global contact_normal;      global jcon_array;         global g_intermediate; 
    global J_st;                global Tau;                global beta;     global alpha;
    global box_vertices_array;  global box_normals_array;  global hfig;
    global num_links_ignore;    global grasped_box;        global choose_visualize;
    global jointspace_dist;     global reach_dist;         global rot_dist;
    global must_achieve_theta_goal; global myVideo;
    
    grasped_obj_con = 0;
    if ~isempty(grasped_box)
        grasped_obj_con = 1;
    end
    
    % Plot local start and goal
    g_init = DQ2Mat(A); g_finl = DQ2Mat(B);
    
    if choose_visualize
        pt_ls = scatter3(g_init(1, 4), g_init(2, 4), g_init(3, 4), 'k', 'filled');
        hold on;
        pt_lg = scatter3(g_finl(1, 4), g_finl(2, 4), g_finl(3, 4), 'k', 'filled');
    end
    
    % Set the bounds of unknowns for PATH Solver
    for j = 1 : (2*dof-num_links_ignore+grasped_obj_con)
        l(j) = -Inf; 
        u(j) = Inf;
    end
    
    l(1, dof+1:end) = 0;   % sice complementarity velocity is always >= 0 
    z = zeros(2*dof-num_links_ignore+grasped_obj_con, 1); % first 7 are joint_q and last 7 are compensating velocities
    q_o = theta0;     % initial config

    % Main loop of iterative motion plan using PATH solver
    currentDQ = A;
    finalDQ = B;
    [g, ~, ~, ~, g_intermediate] = FK_RealBaxter(q_o);
    theta_SCLERP = [];
    theta_SCLERP(:, count) = q_o;
    
    not_reached_flag = true;
    not_achieved_jointspace_goal = true;
    count = 1;

    %/////////////////////////////////////////////////////////////////
    %//////// Achieve T-space and C-space goals simulteneously ///////
    %/////////////////////////////////////////////////////////////////
    while not_reached_flag
        % Compute g_tau and DQ tau    
        [~, ResDQ] = Screw_Lin(currentDQ, finalDQ, Tau);
        % Move end effector to g_tau by approximating joint angles   
        J_st = Jac_RealBaxter(q_o);
        [theta_next, v_ts2js] = theta_next_robust_config(J_st, g, ResDQ, q_o, theta_final, beta, alpha);
        [contact_normal, dista, jcon_array] = get_collision_info4_graspobj(g_intermediate, box_vertices_array, box_normals_array);
        if not_reached_flag
            [z, ~, ~] = pathmcp(z, l, u, 'mcpfuncjacEval');
            q_o = z(1:dof);
        else
            q_o = theta_next;
        end
        
        % Check if q_o is within joint limits
        if ~check_jlimits(q_o)
            fprintf("Joint limit violated. \n");
            break;
        end
        
        [g, ~, ~, ~, g_intermediate] = FK_RealBaxter(q_o);
        currentDQ = Mat2DQ(g);
        count = count+1;
        theta_SCLERP(:, count) = q_o;

        % Check how far from goal
        [rotation_err, position_err] = distDQ(finalDQ, currentDQ);
        
        if position_err < reach_dist && rotation_err < rot_dist
                not_reached_flag = false;
            fprintf("taskspace goal reached according to given tolerance. \n");
        end
    
        jointpsace_err = norm(theta_final - q_o); 
        if jointpsace_err < jointspace_dist
            not_achieved_jointspace_goal = false;
            fprintf("jointspace goal reached according to given tolerance. \n");
        end

        fprintf("iteration=%d\n",count);
        fprintf("pos err: %2.6f, rot err: %2.6f\n", position_err, rotation_err);
        fprintf("jointspace err: %2.6f\n", jointpsace_err);
        fprintf("----------\n");
        
        %/////////////// Plot the motion ///////////////
        if choose_visualize
            plot_motion(hfig);
% % %             plot_motion(hfig, myVideo);
        end
    end
    
    %/////////////////////////////////////////////////////////////////////
    %//// Move in Null(J_st) until C-space goal is achieved completely ///
    %/////////////////////////////////////////////////////////////////////
    th = theta_SCLERP(:, count);
    while not_achieved_jointspace_goal && must_achieve_theta_goal
        J_st = Jac_RealBaxter(th);
        % Inverse of the Jacobian
        inv_J_st = pinv(J_st);
        th = th + alpha*(eye(7,7) - inv_J_st*J_st)*(theta_final - th);
        
        theta_SCLERP(:, count+1) = th;
        count = count + 1;
        
        [g, ~, ~, ~, g_intermediate] = FK_RealBaxter(th);
        currentDQ = Mat2DQ(g);
        [contact_normal, dista, jcon_array] = ...
            get_collision_info4_graspobj(g_intermediate, box_vertices_array, box_normals_array);
        
        [rotation_err, position_err] = distDQ(finalDQ, currentDQ);
        
        jointpsace_err = norm(theta_final - th); 
        if jointpsace_err < jointspace_dist
            not_achieved_jointspace_goal = false;
            fprintf("jointspace goal reached according to given tolerance. \n");
        end

        fprintf("iteration=%d\n",count);
        fprintf("pos err: %2.6f, rot err: %2.6f\n", position_err, rotation_err);
        fprintf("jointspace err: %2.6f\n", jointpsace_err);
        fprintf("----------\n");
        
         %/////////////// Plot the motion ///////////////
        if choose_visualize
            plot_motion(hfig, myVideo);
        end
        
    end
    
    if choose_visualize
        delete(pt_ls); delete(pt_lg);
    end
    
    achieved_goal = g;
    achieved_goalDQ = currentDQ;
    achieved_jointvec = theta_SCLERP(:, end);
    
end