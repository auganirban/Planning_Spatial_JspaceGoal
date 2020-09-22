function [contact_nrml_array, dist_array, jcon_array] = get_collision_info4_graspobj(g_intermediate, box_vertices_array, box_normals_array)
    global contact_pts_array; global grasped_box; global grasped_obj_vertx; 
    global num_links_ignore;  global dof;         global num_obstacles;
    
    [link_cylinder_array] = arm_cylinder_model(g_intermediate);
    
     % /////////////////// Grasped object /////////////////////////
    grasped_obj_con = 0;
    if ~isempty(grasped_box)
        grasped_obj_vertx = g_intermediate(1:3, 1:3, 8)*grasped_box.blk_vrtx' + g_intermediate(1:3, 4, 8);
        grasped_obj_nrml = g_intermediate(1:3, 1:3, 8)*grasped_box.blk_nrm';
        grasped_obj_bvec = cmp_bvec(grasped_obj_nrml, grasped_obj_vertx);
        grasped_obj_con = 1;
    end

    % //////////////// Works related to collision check ////////////////
    dist_array = 1000*ones(1, (dof-num_links_ignore)+grasped_obj_con); 
    contact_nrml_array = zeros(6, (dof-num_links_ignore)+grasped_obj_con); 
    contact_pts_array = zeros(3, 2, (dof-num_links_ignore)+grasped_obj_con);
    jcon_array = zeros(6, 7, (dof-num_links_ignore)+grasped_obj_con);
    
    for box_num = 1:num_obstacles  % loop through the number of obstacles
        box_bvec = cmp_bvec(box_normals_array(:, :, box_num), box_vertices_array(:, :, box_num)); % This can be pre-computed to save time

        % Initial guess to solve nonlinear optimization problem
        % to find the closest distance between the box and arm link
        % (cylinder)
        for itr_index = 1:(dof-num_links_ignore+grasped_obj_con)
            num_link = num_links_ignore + itr_index;
            if num_link <= dof
                current_cylinder = link_cylinder_array{1, num_link};
                x0 = [box_vertices_array(:, 1, box_num); current_cylinder.init_pt];
                % Check collision between box and cylinder
                [x1, x2, min_d] = check_colli(box_normals_array(:, :, box_num), box_bvec, x0, current_cylinder.Trns, current_cylinder.rc, current_cylinder.hc);
            else
                [x1, x2, min_d] = check_colli_boxnbox(box_normals_array(:, :, box_num), box_bvec, grasped_obj_nrml, grasped_obj_bvec);
            end
            
            if dist_array(itr_index) > min_d
                dist_array(itr_index) = min_d;
                contact_nrml_array(1:3, itr_index) = (x2 - x1)/norm(x2 - x1);
                contact_pts_array(:, :, itr_index) = [x1, x2];
            end
        end
    end
    
    % Compute the contact Jacobians
    for i = 1:(dof-num_links_ignore+grasped_obj_con)
        if i <= (dof-num_links_ignore)
            jcon_array(:, :, i) = contact_jacobian2(i+num_links_ignore, contact_pts_array(:, 2, i));
        else
            jcon_array(:, :, i) = contact_jacobian_grasped(contact_pts_array(:, 2, i));
        end
    end
    
end

% check collision between blocks
function [x1, x2, min_d] = check_colli_boxnbox(blk1_N, blk1_b, blk2_N, blk2_b)
    A = [blk1_N' zeros(6, 3); zeros(6, 3) blk2_N'];
    b = [blk1_b; blk2_b];
    f = zeros(6, 1);
    H = diag(ones(1, 6));
    H(1:3, 4:end) = diag(-1*ones(1, 3));
    H(4:end, 1:3) = diag(-1*ones(1, 3));

    opts = optimset('Display', 'off');                      % Processing display off
    [x,fval,exitflag,output,lambda] = quadprog(H, f, A, b, [], [], [], [], [], opts);     % MATLAB's quadprog() function
    x1 = x(1:3); x2 = x(4:6);
    fvalue = 0.5*(x'*H*x) + f'*x;                           % objective value
    min_d = norm(x(1:3)-x(4:6));
%     fprintf('Objective value: %6.8f\n', fvalue);
%     fprintf('Minimum dist between two blocks: %6.8f\n', min_d);
end