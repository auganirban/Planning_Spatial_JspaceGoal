function[frame_array, vertex_array, normal_array] = get_obstacle_wall(window_position)
    global choose_visualize;
    
    box_data = load("box_vrtx_nrml3_exp.mat");                        % Surface normals and vertices of box type obstacle
%     box_data2 = load("box_vrtx_nrml4.mat");                       % Surface normals and vertices of box type obstacle
%     box_data2 = load("box_vrtx_nrml5.mat");                       % Surface normals and vertices of box type obstacle
    box_data2 = load("box_vrtx_nrml6_exp.mat");
    
    % Initialize
    frame_array = zeros(4, 4, 4); vertex_array = zeros(3, 8, 4); normal_array = zeros(3, 6, 4);
    
%     % Compute the transforms of the box frames
%     frame_array(:, :, 1) = [eye(3, 3), [window_position(1); window_position(2); window_position(3)-0.1-0.2]; zeros(1, 3), 1];     % Transformation of box frame in Base frame
%     frame_array(:, :, 2) = [eye(3, 3), [window_position(1); window_position(2)-0.25; window_position(3)-0.1]; zeros(1, 3), 1];    % Transformation of box frame in Base frame
%     frame_array(:, :, 3) = [eye(3, 3), [window_position(1); window_position(2)+0.25; window_position(3)-0.1]; zeros(1, 3), 1];    % Transformation of box frame in Base frame
%     frame_array(:, :, 4) = [eye(3, 3), [window_position(1); window_position(2); window_position(3)+0.1]; zeros(1, 3), 1];         % Transformation of box frame in Base frame

    % Compute the transforms of the box frames (using for experiment)
    frame_array(:, :, 1) = [eye(3, 3), [window_position(1); window_position(2); window_position(3)-(0.215/2)-0.4]; zeros(1, 3), 1];     % Transformation of box frame in Base frame
    frame_array(:, :, 2) = [eye(3, 3), [window_position(1); window_position(2) - (0.666/2) - (0.268/2); window_position(3)-(0.215/2)]; zeros(1, 3), 1];    % Transformation of box frame in Base frame
    frame_array(:, :, 3) = [eye(3, 3), [window_position(1); window_position(2) + (0.666/2) + (0.268/2); window_position(3)-(0.215/2)]; zeros(1, 3), 1];    % Transformation of box frame in Base frame
    frame_array(:, :, 4) = [eye(3, 3), [window_position(1); window_position(2); window_position(3)+(0.215/2)]; zeros(1, 3), 1];         % Transformation of box frame in Base frame

    % Box vertices in base frame
    vertex_array(:, :, 1) = frame_array(1:3, 1:3, 1)*box_data.blk_vrtx' + frame_array(1:3, 4, 1);   % Bottom wall
    vertex_array(:, :, 2) = frame_array(1:3, 1:3, 2)*box_data2.blk_vrtx' + frame_array(1:3, 4, 2);  % Left wall
    vertex_array(:, :, 3) = frame_array(1:3, 1:3, 3)*box_data2.blk_vrtx' + frame_array(1:3, 4, 3);  % Right wall
    vertex_array(:, :, 4) = frame_array(1:3, 1:3, 4)*box_data.blk_vrtx' + frame_array(1:3, 4, 4);   % Upper wall
    
    % Box normals in base frame
    normal_array(:, :, 1) = frame_array(1:3, 1:3, 1)*box_data.blk_nrm';
    normal_array(:, :, 2) = frame_array(1:3, 1:3, 2)*box_data2.blk_nrm';
    normal_array(:, :, 3) = frame_array(1:3, 1:3, 3)*box_data2.blk_nrm';
    normal_array(:, :, 4) = frame_array(1:3, 1:3, 4)*box_data.blk_nrm';

    % Plot the walls
    if choose_visualize
        box3d(vertex_array(:, :, 1), 2);
        box3d(vertex_array(:, :, 2), 2);
        box3d(vertex_array(:, :, 3), 2);
        box3d(vertex_array(:, :, 4), 2);

        draw_frame(frame_array(1:3, 1:3, 1), frame_array(1:3, 4, 1));
        draw_frame(frame_array(1:3, 1:3, 2), frame_array(1:3, 4, 2));
        draw_frame(frame_array(1:3, 1:3, 3), frame_array(1:3, 4, 3));
        draw_frame(frame_array(1:3, 1:3, 4), frame_array(1:3, 4, 4));
    end
end
