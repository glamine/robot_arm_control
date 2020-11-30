function [Rref,xref,omegaref,xdotref] = ComputeReference(atheta,ax,k,t,t0)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%ax = matrix 3*4

xxref = ax(1,1) + ax(1,2)*(t-t0) + ax(1,3)*(t-t0)^2 + ax(1,4)*(t-t0)^3;
xxdotref = ax(1,2) + 2*ax(1,3)*(t-t0) + 3*ax(1,4)*(t-t0)^2;

xyref = ax(2,1) + ax(2,2)*(t-t0) + ax(2,3)*(t-t0)^2 + ax(2,4)*(t-t0)^3;
xydotref = ax(2,2) + 2*ax(2,3)*(t-t0) + 3*ax(2,4)*(t-t0)^2;

xzref = ax(3,1) + ax(3,2)*(t-t0) + ax(3,3)*(t-t0)^2 + ax(3,4)*(t-t0)^3;
xzdotref = ax(3,2) + 2*ax(3,3)*(t-t0) + 3*ax(3,4)*(t-t0)^2;

xref = [xxref;xyref;xzref];
xdotref = [xxdotref;xydotref;xzdotref];

theta = atheta(1) + atheta(2)*(t-t0) + atheta(3)*(t-t0)^2 + atheta(4)*(t-t0)^3;
thetadot = atheta(2) + 2*atheta(3)*(t-t0) + 3*atheta(4)*(t-t0)^2;

eps1 = k(1)*sin(theta/2);
eps2 = k(2)*sin(theta/2);
eps3 = k(3)*sin(theta/2);
eps4 = cos(theta/2);

Rref = [ 1 - 2*eps2*eps2 - 2*eps3*eps3,     2*(eps1*eps2 - eps3*eps4),      2*(eps1*eps3 + eps2*eps4);
             2*(eps1*eps2 + eps3*eps4), 1 - 2*eps1*eps1 - 2*eps3*eps3,      2*(eps2*eps3 - eps1*eps4);
             2*(eps1*eps3 - eps2*eps4),     2*(eps2*eps3 + eps1*eps4), 1 - 2*eps1*eps1 - 2*eps2*eps2];
         
omegaref = thetadot*k;         

end

