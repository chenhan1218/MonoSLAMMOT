function [ output_args ] = plotCameraWires( Xv, color )
%PLOTCAMERAWIRES Summary of this function goes here
%  Detailed explanation goes here

x = [  0.2000 0 0.2000 0 -0.2000 0 -0.2000 -0.2000 0.2000 0.2000 -0.2000 ];
y = [ 0.2000 0 -0.2000 0 -0.2000 0 0.2000 -0.2000 -0.2000 0.2000 0.2000 ];
z = [  0.4000 0 0.4000 0 0.4000 0 0.4000 0.4000 0.4000 0.4000 0.4000 ];

x = x.*2;
y = y.*2;
z = z.*2;

% x = x + ones(size(x))*Xv(1);
% y = y + ones(size(y))*Xv(2);
% z = z + ones(size(z))*Xv(3);

points = [ x; y; z ];

R_wc = q2r( Xv(4:7) );
points = R_wc*points;

points = points + [ ones(1,size(points,2))*Xv(1); ones(1,size(points,2))*Xv(2); ones(1,size(points,2))*Xv(3)];

x = points(1,:);
y = points(2,:);
z = points(3,:);

plot3(x,y,z, '-', 'Color', color);