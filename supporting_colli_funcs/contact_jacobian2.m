function [jcon] = contact_jacobian2(n,conpt)
    global J_st;

    P_hat=[0,-conpt(3),conpt(2); 
           conpt(3),0,-conpt(1); 
          -conpt(2),conpt(1),0];
    Js = zeros(6, 7);
    Js(:, 1:n) = J_st(:, 1:n);
    temp = [eye(3,3) -P_hat]*Js;
    jcon = [temp; zeros(3, size(temp, 2))];
end