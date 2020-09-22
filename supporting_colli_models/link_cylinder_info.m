function [cylinder_struc] = link_cylinder_info(T, h, r, initial_pt)
    %% Returns a structure with fileds T, h, r
    cylinder_struc.Trns = T;
    cylinder_struc.rc = r;
    cylinder_struc.hc = h;
    cylinder_struc.init_pt = initial_pt;
end