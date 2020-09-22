function [Q] = Mat2DQ(M)
    % Matrix to Dual Quaternion
    Q_real = rotm2quat(M(1:3, 1:3));
    Q_dual = QMult([0, M(1:3, 4)'], Q_real)/2;
    Q = [Q_real, Q_dual];
end