function [m] = RotZ(q)
%RotZ returns homogeneous transformation matrix around z axis with the angle of q
m = [cos(q), -sin(q), 0, 0; ...
    sin(q), cos(q), 0, 0;...
    0, 0, 1, 0; ...
    0, 0, 0, 1];
end