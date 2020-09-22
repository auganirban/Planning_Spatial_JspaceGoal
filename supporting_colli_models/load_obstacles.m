function[frame_array, vertex_array, normal_array] = load_obstacles(obs_position)
    global choose_visualize; 
    global num_obstacles; 
    global obstacle_box_data;    
    
    % Initialize
    frame_array = zeros(4, 4, num_obstacles); 
    vertex_array = zeros(3, 8, num_obstacles); 
    normal_array = zeros(3, 6, num_obstacles);
    
    for i = 1:num_obstacles
        % Compute the transforms of the box frames (using for experiment)
        frame_array(:, :, i) = [eye(3, 3), [obs_position(1,i); obs_position(2,i); obs_position(3,i)]; zeros(1, 3), 1];     % Transformation of box frame in Base frame
        
        % Box vertices in base frame
        vertex_array(:, :, i) = frame_array(1:3, 1:3, i)*obstacle_box_data{i}.blk_vrtx' + frame_array(1:3, 4, i);   % obstacle
        
        % Box normals in base frame
        normal_array(:, :, i) = frame_array(1:3, 1:3, i)*obstacle_box_data{i}.blk_nrm';
        
        % Plot the walls
        if choose_visualize
            box3d(vertex_array(:, :, i), 2);
            draw_frame(frame_array(1:3, 1:3, i), frame_array(1:3, 4, i));
        end
    end
    
end

