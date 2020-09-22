function [x1, x2, min_d] = check_colli(blk1_N, blk1_b, x0, cynT, cynr, cynh)
    %% check collision between box and cylinder 
    Rbox = cynT(1:3, 1:3); pbox = cynT(1:3, 4);
    
    % Objective function to minimize
    obj = @(x)norm(x(1:3) - x(4:6))^2;
    
    % Build-up the constraints
    A = [blk1_N', zeros(6, 3); zeros(2, 3), [Rbox(:, 3)'; -Rbox(:, 3)']];
    b = [blk1_b; cynh + Rbox(:, 3)'*pbox; -Rbox(:, 3)'*pbox];
    opts = optimoptions('fmincon','Algorithm','interior-point', 'Display', 'off');
    
    % Call MATLAB's built-in fmincon function
    x = fmincon(obj, x0, A, b, [], [], [], [], @(x)disk_constraint(x, Rbox, pbox, cynr), opts);
    
    % Separate the points on the gripper plate and the peg
    x1 = x(1:3);
    x2 = x(4:6);
    
    % Find the closest distance between the two objects
    min_d = norm(x1 - x2);
end
