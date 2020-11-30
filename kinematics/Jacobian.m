function [xdot] = Jacobian(q,qdot)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%jacobian 5*5

%possibilités pour jacobian inverse : 
%compute 6*6 puis inverse
%exprimer dans autre repère et supprimer ligne : 5*5
%hack : gerer les deux separement rot et trans

t1 = q(1);
t2 = q(2);
t3 = q(3);
t4 = q(4);
%t5 = q(5);

J = [ -sin(t1)*(a1 + a3*cos(t2 + t3) + a2*cos(t2) - d5*sin(t2 + t3 + t4)), -cos(t1)*(a3*sin(t2 + t3) + a2*sin(t2) + d5*cos(t2 + t3 + t4)), -cos(t1)*(a3*sin(t2 + t3) + d5*cos(t2 + t3 + t4)), -d5*cos(t2 + t3 + t4)*cos(t1),                     0;
  cos(t1)*(a1 + a3*cos(t2 + t3) + a2*cos(t2) - d5*sin(t2 + t3 + t4)), -sin(t1)*(a3*sin(t2 + t3) + a2*sin(t2) + d5*cos(t2 + t3 + t4)), -sin(t1)*(a3*sin(t2 + t3) + d5*cos(t2 + t3 + t4)), -d5*cos(t2 + t3 + t4)*sin(t1),                          0;
                                                                   0,            d5*sin(t2 + t3 + t4) - a2*cos(t2) - a3*cos(t2 + t3),            d5*sin(t2 + t3 + t4) - a3*cos(t2 + t3),          d5*sin(t2 + t3 + t4),                          0;
                                                                   0,                                                       -sin(t1),                                          -sin(t1),                      -sin(t1), -sin(t2 + t3 + t4)*cos(t1);
                                                                   0,                                                        cos(t1),                                           cos(t1),                       cos(t1), -sin(t2 + t3 + t4)*sin(t1);
                                                                   1,                                                              0,                                                 0,                             0,         -cos(t2 + t3 + t4)];

xdot = J*qdot;
                                                               
end

