function [m] = TransZ(a)
%TransZ returns homogeneous transformation matrix moving along with z axis with the disntance of a
m = [1, 0, 0, 0;...
     0, 1, 0, 0;...
     0, 0, 1, a; ...
     0, 0, 0, 1];
end