function [f, J, domerr]= mcpfuncjacEval(z,jacflag)
    
    global contact_normal; global dista; global q_o; global v_ts2js;
    global jcon_array;     global dof;   global h;   global safe_dist;
    global grasped_box;    global num_links_ignore;

    grasped_obj_con = 0;
    if ~isempty(grasped_box)
        grasped_obj_con = 1;
    end
    num_unknowns = 2*dof - num_links_ignore + grasped_obj_con;
    
    f = zeros(num_unknowns, 1); J=zeros(num_unknowns, num_unknowns); domerr = 0;
    invcon_array = zeros(dof, 6, dof - num_links_ignore + grasped_obj_con); 
    q_n = z(1:dof); 
      
    add_element = zeros(dof, 1); 
    temp_new = zeros(dof - num_links_ignore + grasped_obj_con, 1);
    first_part_mat = zeros(dof, dof - num_links_ignore + grasped_obj_con); 
    second_part_mat = zeros(dof - num_links_ignore + grasped_obj_con, dof);
    
    for num = 1:dof - num_links_ignore + grasped_obj_con
        num_offset = num_links_ignore + num;
        temp_jac = jcon_array(:, :, num);
        if ~isempty(grasped_box) && num <= (dof-num_links_ignore)
            invcon_array(1:num_offset, 1:3, num) = pinv(temp_jac(1:3, 1:num_offset));
        else
            invcon_array(1:num_offset-1, 1:3, num) = pinv(temp_jac(1:3, 1:num_offset-1));
        end
        % invcon_array2(:, :, num) = pinv(jcon_array(:, :, num)); % computations can be saved here
% % %         invcon_array(1:num, 1:3, num) = pinv(temp_jac(1:3, 1:num));
        add_element = add_element + invcon_array(:, :, num)*contact_normal(:, num)*z(dof+num);
        temp_new(num, 1) = contact_normal(:, num)'*jcon_array(:, :, num)*(q_n-q_o);
        first_part_mat(:, num) = invcon_array(:, :, num)*contact_normal(:, num);
        second_part_mat(num, :) = contact_normal(:, num)'*jcon_array(:, :, num);
    end

    f(1:dof) = (q_o - q_n) + v_ts2js + h*add_element;
    f(dof+1 : end) = temp_new + (dista' - safe_dist);
    
    if (jacflag)
        for i = 1:dof
            J(i, i) = -1;
        end
        J(1:dof, dof+1:end) = h*first_part_mat;
        J(dof+1:end, 1:dof) = second_part_mat;
        J=sparse(J);
    end
end