function [spatial_jac, analytical_jac] = JS_RealBaxter(th)
    dim = 3;
    num_of_joints = 7;
    spatial_jac = zeros(dim+3, num_of_joints);
    
%     [gst, transform_upto_joint] = baxter_forward_kinematics(gst0, joint_axes, q_axes, theta);
    [gst, transform_upto_joint, joint_axes, q_axes, ~] = FK_RealBaxter(th);
    
    for i = 1:num_of_joints
        if i > 1
		    g = transform_upto_joint(:,:,i);
		    R = g(1:3,1:3);
		    p = g(1:3,4);
		    p_hat = [0 -p(3) p(2);
		                 p(3) 0 -p(1);
		                -p(2) p(1) 0];
		    temp1 = p_hat*R;
		    Ad_g = [R temp1];
		    temp2 = [zeros(3,3) R];
		    Ad_g = [Ad_g; temp2];
        end
        
        omega = joint_axes(:,i);
        q = q_axes(:,i);
        xi = [cross(-omega, q); omega];

        if i > 1
            xi_prime = Ad_g*xi;
        else
            xi_prime = xi;
        end
        spatial_jac(:,i) =  xi_prime;
    end
    p_hat_gst = [0 -gst(3, 4) gst(2, 4); gst(3, 4) 0 -gst(1, 4); -gst(2, 4) gst(1, 4) 0];
    analytical_jac = [eye(3), p_hat_gst; zeros(3), eye(3)]*spatial_jac;
end