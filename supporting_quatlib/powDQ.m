function [q] = powDQ(Q,t)
    Ar = Q(1, 1:4); Ad = Q(1, 5:8);
    axang = quat2axang(Ar);
    ax = axang(1:3);
    ang = axang(4);
    
    Ar_conj = [Ar(1, 1), -Ar(1, 2:4)];
    t_quat = 2 * QMult(Ad, Ar_conj);
    t_vec = t_quat(2:4);
    if ang ~= 0
        d = t_vec * ax';
        m = (cross(t_vec, ax) + (t_vec - d * ax) * cot(ang/2))/2;
    else
        m = zeros(1, 3);
        ax = t_vec/norm(t_vec);
        d = t_vec * ax';
    end
    Rr = [cos(t*ang/2), ax*sin(t*ang/2)];
    Rd = [-t*d*sin(t*ang/2)/2, t*d*cos(t*ang/2)*ax/2 + m*sin(t*ang/2)];
    q = [Rr, Rd];
end
