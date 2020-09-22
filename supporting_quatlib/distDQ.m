function [rot_err, pos_err] = distDQ(q1, q2)
    % Value of rotation metric
    rot_err = min(norm(q1(1, 1:4) - q2(1, 1:4)), norm(q1(1, 1:4) + q2(1, 1:4)));
    
    % Value of position metric
    Ar_conj1 = [q1(1, 1), -q1(1, 2:4)];
    t_quat1 = 2 * QMult(q1(1, 5:8), Ar_conj1);
    
    Ar_conj2 = [q2(1, 1), -q2(1, 2:4)];
    t_quat2 = 2 * QMult(q2(1, 5:8), Ar_conj2);
    
    pos_err = norm(t_quat1(2:4) - t_quat2(2:4));
%     t_quat1
%     t_quat2
% % %     fprintf("Position Error: %2.6f, Rotation Error: %2.6f \n", pos_err, rot_err);
end

% Anik's implementation
% Distance metric for rotation.
%     q1r = q1(:,1:4);
%     q2r = q2(:,1:4);
%     s = q1-q2;
%     a = q1+q2;
%     n1 = sqrt(s(1,1)^2+s(1,2)^2+s(1,3)^2+s(1,4)^2);
%     n2 = sqrt(a(1,1)^2+a(1,2)^2+a(1,3)^2+a(1,4)^2);
%     N = [n1 n2];
%     m = min(N);
% % Error in position between two poses
% p1 = DQuaternionToMatrix(q1);
% p2 = DQuaternionToMatrix(q2);
% e =  norm(p2 - p1);