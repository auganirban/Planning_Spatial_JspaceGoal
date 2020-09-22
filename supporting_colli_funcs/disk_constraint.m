function [c,ceq] = disk_constraint(x, Rbox, pbox, rc)
    %% circular cross section constraint of the peg to be used with fmincon
    % global baseTcyn; global rc;
%     Rbox = baseTcyn(1:3, 1:3); pbox = baseTcyn(1:3, 4);
    temp1 = Rbox(:, 1)'*(x(4:6) - pbox);
    temp2 = Rbox(:, 2)'*(x(4:6) - pbox);
    c = temp1^2 + temp2^2 - (rc)^2;
    ceq = [];
end