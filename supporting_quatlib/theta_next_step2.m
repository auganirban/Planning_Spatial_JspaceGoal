function [thetha_next, ip_task2js] = theta_next_step2( J_s, g_t, result, theta_t, beta )
    Q1 = rotm2quat(g_t(1:3,1:3));
    p1 = g_t(1:3, 4);
    gamma_t = [p1; Q1'];

    g_th = DQ2Mat(result);
    Q2 = result(1, 1:4);
    p2 = g_th(1:3, 4);
    gamma_th = [p2; Q2'];

    % Define p hat as a skew symmetric form of p1
    ph = [0, -p1(3),p1(2);
          p1(3), 0, -p1(1);
         -p1(2), p1(1),0];

    % Define J1 in terms of Q1
    J1 = [-Q1(2), Q1(1),-Q1(4), Q1(3);
          -Q1(3), Q1(4), Q1(1),-Q1(2);
          -Q1(4),-Q1(3), Q1(2), Q1(1)];

    % Define J2
    J2 = [eye(3), 2*ph*J1;
          zeros(3), 2*J1];

    B = J_s'*((J_s*J_s')\J2);
    ip_task2js = beta * B * (gamma_th - gamma_t);
    
    % Check all joint angle changes are within a threshold
    dth = threshold_dtheta(ip_task2js);
    ip_task2js = dth;
    thetha_next = theta_t + dth;
end

function [dth_bounded] = threshold_dtheta(dth_raw)
    dth_bounded = dth_raw; dth_tol = 0.01;
    for i = 1:7
        if dth_bounded(i) > dth_tol
            dth_bounded(i) = dth_tol;
        end
        if dth_bounded(i) < -dth_tol
            dth_bounded(i) = -dth_tol;
        end
    end
end