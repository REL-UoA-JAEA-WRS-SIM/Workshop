function [m] = DH(alpha, a, d, q)
%DH returns homogeneous transformation matrix following (modified)DH method and parameters
m = RotX(alpha) * TransX(a) * RotZ(q) * TransZ(d);
end