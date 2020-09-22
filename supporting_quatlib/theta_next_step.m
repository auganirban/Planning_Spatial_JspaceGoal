function thetha_next = theta_next_step( J_s, g_t, result, theta_t, beta )
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
    J1 = [-Q1(2), Q1(1), -Q1(4), Q1(3);
          -Q1(3), Q1(4), Q1(1), -Q1(2);
          -Q1(4), -Q1(3), Q1(2), Q1(1)];

    % Define J2
    J2 = [eye(3), 2*ph*J1;
          zeros(3), 2*J1];

    B = J_s'*((J_s*J_s')\J2);      

    thetha_next = theta_t + beta * B * (gamma_th - gamma_t);
end