function poly_rectangle(p1, p2, p3, p4, ele)
    %% Function that plots the gripper plates
    % The points must be in the correct sequence.
    % The coordinates must consider x, y and z-axes.
    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
    if ele == 1
        colr = [0.9, 0, 0];
    else
        colr = [0.5, 0.5, 0.5];
    end
    fill3(x, y, z, colr, 'Facealpha', 0.7);
    hold on
end