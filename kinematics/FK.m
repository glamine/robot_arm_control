function [T50] = FK(q,param)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

a1 = param(1);
a2 = param(2);
a3 = param(3);
d1 = param(4);
d5 = param(5);

t1 = q(1);
t2 = q(2);
t3 = q(3);
t4 = q(4);
t5 = q(5);

T50 = [ sin(t1)*sin(t5) + cos(t2 + t3 + t4)*cos(t1)*cos(t5),   cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5), -sin(t2 + t3 + t4)*cos(t1), cos(t1)*(a1 + a3*cos(t2 + t3) + a2*cos(t2) - d5*sin(t2 + t3 + t4));
        cos(t2 + t3 + t4)*cos(t5)*sin(t1) - cos(t1)*sin(t5), - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5), -sin(t2 + t3 + t4)*sin(t1), sin(t1)*(a1 + a3*cos(t2 + t3) + a2*cos(t2) - d5*sin(t2 + t3 + t4));
                                 -sin(t2 + t3 + t4)*cos(t5),                             sin(t2 + t3 + t4)*sin(t5),         -cos(t2 + t3 + t4),           d1 - a3*sin(t2 + t3) - a2*sin(t2) - d5*cos(t2 + t3 + t4);
                                                          0,                                                     0,                          0,                                                                  1];


end

