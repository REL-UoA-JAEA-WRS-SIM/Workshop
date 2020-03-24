function [p] = DrawJacoArmPose3DwitHandFrame(p, T06)
%DrawJacoArmPose3D Draw Jaco arm pose in 3D plot
%   Input
%       p = [x0, x1, x2, x3, x4, x5, x6;
%            y0, y1, y2, y3, y4, y5, y6; 
%            z0, z1, z2, z3, z4, z5, z6]; 

% Extract x, y, z componet as an array 
x = p(1,:);
y = p(2,:);
z = p(3,:);

% Draw a pose of robot arm in 3D
view(3);
plot3(x, y, z, 'k-o');
% Draw hand frame axis 
o = T06*[0; 0; 0; 1];
xAxis = T06*[0.1;   0;   0; 1];
yAxis = T06*[  0; 0.1;   0; 1];
zAxis = T06*[  0;   0; 0.1; 1];
plot3([o(1),xAxis(1)], [o(2),xAxis(2)], [o(3),xAxis(3)], 'r-');
plot3([o(1),yAxis(1)], [o(2),yAxis(2)], [o(3),yAxis(3)], 'g-');
plot3([o(1),zAxis(1)], [o(2),zAxis(2)], [o(3),zAxis(3)], 'b-');

% Plot format
xlim([-0.5, 1.5]);
ylim([-0.5, 1.5]);
zlim([-0.5, 1.5]);
title('Jaco arm pose in 3D');
pbaspect([1 1 1]);
end
