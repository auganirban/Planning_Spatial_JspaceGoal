function[] =  plot_motion_video(hfig, myVideo)
% function[] =  plot_motion_video(hfig)
    global link1_cylinder1; global link2_cylinder2; global link3_cylinder3;
    global link4_cylinder4; global link5_cylinder5; global link6_cylinder6;
    global link7_cylinder7; global link8_cylinder8; global link9_cylinder9;
    global link10_cylinder10; global n_pts; global g; global contact_pts_array;
    global count; global grasped_obj_vertx; global grasped_box;
    
    %/////////////////////////////////////////////////////////////
    %///////////////////// Plotting //////////////////////////////
    %/////////////////////////////////////////////////////////////
    % Plot the links as cylinders
    link1 = surf(reshape(link1_cylinder1(1, :), [2, n_pts]), reshape(link1_cylinder1(2, :), [2, n_pts]), reshape(link1_cylinder1(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    hold on;
    offset1 = surf(reshape(link2_cylinder2(1, :), [2, n_pts]), reshape(link2_cylinder2(2, :), [2, n_pts]), reshape(link2_cylinder2(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link2 = surf(reshape(link3_cylinder3(1, :), [2, n_pts]), reshape(link3_cylinder3(2, :), [2, n_pts]), reshape(link3_cylinder3(3, :), [2, n_pts]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link3 = surf(reshape(link4_cylinder4(1, :), [2, n_pts]), reshape(link4_cylinder4(2, :), [2, n_pts]), reshape(link4_cylinder4(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    offset2 = surf(reshape(link5_cylinder5(1, :), [2, n_pts]), reshape(link5_cylinder5(2, :), [2, n_pts]), reshape(link5_cylinder5(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link4 = surf(reshape(link6_cylinder6(1, :), [2, n_pts]), reshape(link6_cylinder6(2, :), [2, n_pts]), reshape(link6_cylinder6(3, :), [2, n_pts]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link5 = surf(reshape(link7_cylinder7(1, :), [2, n_pts]), reshape(link7_cylinder7(2, :), [2, n_pts]), reshape(link7_cylinder7(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    offset3 = surf(reshape(link8_cylinder8(1, :), [2, n_pts]), reshape(link8_cylinder8(2, :), [2, n_pts]), reshape(link8_cylinder8(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link6 = surf(reshape(link9_cylinder9(1, :), [2, n_pts]), reshape(link9_cylinder9(2, :), [2, n_pts]), reshape(link9_cylinder9(3, :), [2, n_pts]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link7 = surf(reshape(link10_cylinder10(1, :), [2, n_pts]), reshape(link10_cylinder10(2, :), [2, n_pts]), reshape(link10_cylinder10(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');

%     % Draw the frames
%     frame_obj_arr = [];
%     for i = 1:8
%         T = g_intermediate(:, :, i);
%         frame_obj_arr(i) = draw_frame(T(1:3, 1:3), T(1:3, 4));
%     end
    
%     title("Baxter left arm model");
%     xlabel("x [m]");
%     ylabel("y [m]");
%     zlabel("z [m]");
%     view(164, 47);
    
    % Comment this part if you do not need to visualize the closest points.
    count_grasped = 0;
    if ~isempty(grasped_box)
        count_grasped = 1;
    end
% % %     line_obj_arr = []; obj_count = 1;
% % %     for i = 1:7+count_grasped
% % %         cnt_pts = contact_pts_array(:, :, i);
% % % 
% % %         % Plot the closest point pair
% % %         line_obj_arr(obj_count) = plot3([cnt_pts(1, 1), cnt_pts(1, 2)], [cnt_pts(2, 1), cnt_pts(2, 2)], [cnt_pts(3, 1), cnt_pts(3, 2)], 'ko');
% % %         line_obj_arr(obj_count+1) = plot3([cnt_pts(1, 1), cnt_pts(1, 2)], [cnt_pts(2, 1), cnt_pts(2, 2)], [cnt_pts(3, 1), cnt_pts(3, 2)], 'g');
% % %         obj_count = obj_count + 2;
% % %     end
    
%     % Plot end effector poses
%     if mod(count, 10) == 0
%         quiver3(g(1, 4), g(2, 4), g(3, 4), g(1, 1), g(2, 1), g(3, 1), 0.05, 'r');
%         quiver3(g(1, 4), g(2, 4), g(3, 4), g(1, 2), g(2, 2), g(3, 2), 0.05, 'g');
%         quiver3(g(1, 4), g(2, 4), g(3, 4), g(1, 3), g(2, 3), g(3, 3), 0.05, 'b');
%     end

    if ~isempty(grasped_obj_vertx)
        grasp_plane1 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 4), 1);
        grasp_plane2 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 5), grasped_obj_vertx(:, 8), 1);
        grasp_plane3 = poly_rectangle2(grasped_obj_vertx(:, 5), grasped_obj_vertx(:, 6), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 8), 1);
        grasp_plane4 = poly_rectangle2(grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 4), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 6), 1);
        grasp_plane5 = poly_rectangle2(grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 6), grasped_obj_vertx(:, 5), 1);
        grasp_plane6 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 4), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 8), 1);
    end
    
    drawnow;
    
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    
    % delete drawn objects
    delete(link1); delete(link2); delete(link3); delete(link4); delete(link5);
    delete(link6); delete(link7); delete(offset1); delete(offset2); delete(offset3);
    
% % %     for i = 1:obj_count-1
% % %         delete(line_obj_arr(i));
% % %     end
    
    if ~isempty(grasped_obj_vertx)
        delete(grasp_plane1); delete(grasp_plane2); delete(grasp_plane3);
        delete(grasp_plane4); delete(grasp_plane5); delete(grasp_plane6);
    end
%     
% %     for i = 1:8
% %         delete(frame_obj_arr(i));
% %     end
% %     drawnow;
end