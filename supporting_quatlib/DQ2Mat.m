function [T] = DQ2Mat(Qm)
    Ar = Qm(1, 1:4); Ad = Qm(1, 5:8);
    rot_mat = quat2rotm(Ar);    
    Ar_conj = [Ar(1, 1), -Ar(1, 2:4)];
    t_quat = 2 * QMult(Ad, Ar_conj);
    t_vec = t_quat(2:4);
    T = [rot_mat, t_vec'; 0, 0, 0, 1];
end

% Anik's implementation
%     %Dual Quaternion to Matrix
%     % Takes input of dual quaternion Qm  = Qr+ E Qd
%     for i =1 : size(Qm, 1)    
%         q0 = Qm(i,1);
%         q1 = Qm(i,2);
%         q2 = Qm(i,3);
%         q3 = Qm(i,4);
%         M (1,1) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
%         M (1,2) = (2*q1*q2 - 2*q0*q3);
%         M (1,3) = 2*q1*q3 + 2*q0*q2;
%         M (2,1) = (2*q1*q2 + 2*q0*q3);
%         M (2,2) = q0*q0 + q2*q2 - q1*q1 - q3*q3;
%         M (2,3) = 2*q2*q3 - 2*q0*q1;
%         M (3,1) = 2*q1*q3 - 2*q0*q2;
%         M (3,2)=  2*q2*q3 + 2*q0*q1;
%         M (3,3)= q0*q0 + q3*q3 - q1*q1 - q2*q2;
% 
%         p = Qm(i,5);
%         u = Qm(i,6);
%         r = Qm(i,7);
%         s = Qm(i,8);
% 
%         q_d = [p u r s];
%         q_d_v = [u r s];
%         q_r_conj = [q0 -q1 -q2 -q3];
%         q_r_conj_v = [-q1 -q2 -q3];
%         f = QMult(q_d, q_r_conj); % q = q_1 +e q_2
% 
%         t = 2*f;
%         t = t';
%         t = t(2:4,:);
% 
%         T(:,:,i) = [M t;0 0 0 1];
%     end




