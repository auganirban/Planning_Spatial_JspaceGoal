function [Q_conj] = DQConjugate(Q)
    % Conjugate of a Dual quaternion is q* = q_r - E(q_d)
    % Hence given a dual quaternion (1X8) Matrix we know last four terms are
    % the the dual part of the quaternion.
    Q_conj = [Q(1,1), -Q(1,2:4), Q(1,5), -Q(1,6:8)];
end

