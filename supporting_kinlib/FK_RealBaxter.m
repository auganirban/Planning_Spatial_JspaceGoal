function [gst, transform_upto_joint, joint_ax, joint_q, gst_intermediate] = FK_RealBaxter(th)
    %//////////////////////////////////////////////////////////
    %////////////  Joint information for left arm /////////////
    %//////////////////////////////////////////////////////////
    % Joint origins wrt base frame (Needs to be Updated)
    q = [0.065, 0.258, 0.119;
        0.114, 0.307, 0.389;
        0.188, 0.377, 0.390;
        0.377, 0.560, 0.324;
        0.452, 0.632, 0.323;
        0.648, 0.819, 0.312;
        0.720, 0.909, 0.304;
        0.898, 1.062, 0.313]'; % s0, s1, e0, e1. w0, w1, w2, left_gripper(standard_narrow finger)
    
    % Joint orientations wrt base frame. Quat = [q0, q1, q2, q3] format.
    Orientation = [0.927, -0.001, 0.000, 0.374;
         0.656, -0.655, -0.268, 0.261;
         0.660, -0.265, 0.652, 0.264;
         0.653, -0.658, -0.263, 0.266;
         0.653, -0.268, 0.656, 0.261;
         0.654, -0.658, -0.267, 0.262;
         0.650, -0.274, 0.658, 0.265;
         0.658, -0.266, 0.653, 0.263]; % s0, s1, e0, e1. w0, w1, w2, gripper_base
    
    % Rotation matrix
    R = quat2rotm(Orientation); % Orientation in matrix form
    
    % Define gst0 of end_effector
    g0 = [R(:, :, 8), q(:, 8); 0 0 0 1];
    
    joint_ax = zeros(3, 7);
    for i = 1:7
        joint_ax(:, i) = R(:, 3, i);
    end
    joint_q = q(:, 1:end-1);
    
    [gst, transform_upto_joint] = direct_kin(g0, joint_ax, joint_q, th);
    
    % Compute transform of intermediate frames
    gst_intermediate = zeros(4, 4, 8);
    for i = 1:7
        gst_intermediate(:, :, i) = transform_upto_joint(:, :, i+1) * [R(:, :, i), q(:, i); 0, 0, 0, 1];
    end
    gst_intermediate(:, :, end) = gst;
    
%     gst_intermediate = zeros(4, 4, 8);
%     for i = 1:7
%         [e(:,:,i), xi(:,:,i), xih(:,:,i)] = expon(w(:,i), q(:,i), theta(i));
%         g_ab = g_ab * e(:,:,i);
%         gst_intermediate(:, :, i) = g_ab * [R(:,:,i), q(:,i); 0, 0, 0, 1];
%     end
%     g_st = g_ab * g0;
%     gst_intermediate(:, :, end) = g_st;
     
end