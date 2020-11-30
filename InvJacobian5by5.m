function [qdotcommand] = InvJacobian5by5(xdot,q,param)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%ATTENTION xdot must be expressed in referential 1 !!
%note  : pas de prise en compte d'une commande de omega sur x1.
%supposée zero tout le temps 

a1 = param(1);
a2 = param(2);
a3 = param(3);
%d1 = param(4);
d5 = param(5);

%t1 = q(1);
t2 = q(2);
t3 = q(3);
t4 = q(4);
%t5 = q(5);


invJ5by5 = [                                    0,                                               0,                     1/(a1 + a3*cos(t2 + t3) + a2*cos(t2) - d5*sin(t2 + t3 + t4)),                   0,                                                    0;
                       cos(t2 + t3)/(a2*sin(t3)),                       sin(t2 + t3)/(a2*sin(t3)),                                                                                0,                   0,                            (d5*cos(t4))/(a2*sin(t3));
 -(a3*cos(t2 + t3) + a2*cos(t2))/(a2*a3*sin(t3)), -(a3*sin(t2 + t3) + a2*sin(t2))/(a2*a3*sin(t3)),                                                                                0,                   0, -(d5*(a2*cos(t3 + t4) + a3*cos(t4)))/(a2*a3*sin(t3));
                            cos(t2)/(a3*sin(t3)),                            sin(t2)/(a3*sin(t3)),                                                                                0,                   0,          (d5*cos(t3 + t4) + a3*sin(t3))/(a3*sin(t3));
                                               0,                                               0, 1/(cos(t2 + t3 + t4)*(a1 + a3*cos(t2 + t3) + a2*cos(t2) - d5*sin(t2 + t3 + t4))), 1/cos(t2 + t3 + t4),                                                    0];
                                                                                    

qdotcommand = invJ5by5*xdot([1 2 3 5 6]);
                                           
end

