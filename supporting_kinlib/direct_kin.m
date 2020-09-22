function [gst, transform_upto_joint] = direct_kin(gst0, joint_axes, q_axes, theta)
    % This code computes direct-kinematics of a robotic open chain
    % manipulator using product-of-exponential-of-twist formula.
    % Anirban Sinha, State University of New York, Stony Brook
    % Updated: June 15th., 2019
    dim = 3;
    num_of_joints = length(theta);
    gst_temp = eye(dim+1,dim+1);

    transform_upto_joint = zeros(dim+1, dim+1, num_of_joints+1);
    for i = 1:num_of_joints 
        transform_upto_joint(:,:,i) = gst_temp;
            omega = joint_axes(:,i);
            q = q_axes(:,i);
            xi = [cross(-omega, q); omega];
        gst_joint_i = exp_twist(xi, theta(i));
        gst_temp = gst_temp*gst_joint_i;
    end
    transform_upto_joint(:,:,end) = gst_temp;
    gst = gst_temp*gst0;
end