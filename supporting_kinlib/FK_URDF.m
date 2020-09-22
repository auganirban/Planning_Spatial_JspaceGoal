function [gst, transform_upto_joint, joint_ax, joint_q, gst_intermediate] = FK_URDF(th)
    %//////////////////////////////////////////////////////////
    %////////////  Joint information for left arm /////////////
    %//////////////////////////////////////////////////////////
    % Joint origins wrt base frame (Needs to be Updated)
    q = [0.064, 0.259, 0.130;
        0.113, 0.308, 0.400;
        0.185, 0.380, 0.400;
        0.371, 0.566, 0.331;
        0.444, 0.639, 0.331;
        0.635, 0.830, 0.321;
        0.717, 0.912, 0.321;
        0.882, 1.077, 0.321]'; % s0, s1, e0, e1. w0, w1, w2, left_gripper(standard_narrow finger)
    
    % Joint orientations wrt base frame. Quat = [q0, q1, q2, q3] format.
    Orientation = [0.924, 0.000, 0.000, 0.383;
                   0.653, -0.653, -0.271, 0.271;
                   0.653, -0.271, 0.653, 0.271;
                   0.653, -0.653, -0.271, 0.271;
                   0.653, -0.271, 0.653, 0.271;
                   0.653, -0.653, -0.271, 0.271;
                   0.653, -0.271, 0.653, 0.271;
                   0.653, -0.271, 0.653, 0.271]; % s0, s1, e0, e1. w0, w1, w2, gripper_base
    
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
     
end
