function [plane_obj] = poly_rectangle2(p1, p2, p3, p4, ele)
    % The points must be in the correct sequence.
    % The coordinates must consider x, y and z-axes.
    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
    if ele == 1
%         colr = 'r';
        colr = [0.1, 0.6, 0.9];
    else
        colr = 'b';
    end
    plane_obj = fill3(x, y, z, colr, 'Facealpha', 0.7);
    hold on
end