function [checked_ok] = check_jlimits(th)
    checked_ok = true;
    
    % These limits are obtainedfrom the following website
    % (https://www.ohio.edu/mechanical-faculty/williams/html/pdf/BaxterKinematics.pdf)
    up_lim = [51, 60, 173, 150, 175, 120, 175]*(pi/180);
    lw_lim = [-141, -123, -173, -3, -175, -90, -175]*(pi/180);
    
    % Check limit for joint1
    for i = 1:7
        if th(i) <= lw_lim(i) || th(i) >= up_lim(i)
            checked_ok = false;
            break;
        end
    end
end