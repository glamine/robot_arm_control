function [q] = IK(o,R,param)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

a1 = param(1);
a2 = param(2);
a3 = param(3);
d1 = param(4);
d5 = param(5);

%

r11 = R(1,1);
r12 = R(1,2); 
r13 = R(1,3);
r21 = R(2,1);
r22 = R(2,2);
r23 = R(2,3);
%r31 = R(3,1);
%r32 = R(3,2);
r33 = R(3,3);

Oc = o - d5.*R*[0;0;1];

xc = Oc(1);
yc = Oc(2);
zc = Oc(3);

%THETA1
theta1_ik = atan2(yc,xc);

r = sqrt(xc*xc + yc*yc);
t = r - a1;
s = d1 - zc;
D = (t*t + s*s - a2*a2 - a3*a3)/(2*a2*a3);

%THETA3
theta3_ik = atan2(sqrt(1-D*D),D);
%+- ? UP and DOWN check
 
c1ik = cos(theta3_ik);
s1ik = sin(theta3_ik);
c3ik = cos(theta3_ik);
s3ik = sin(theta3_ik);

%THETA2
theta2_ik =  atan2(s,t) - atan2(a3*s3ik,a2+a3*c3ik);

c23ik = cos(theta2_ik + theta3_ik);
s23ik = sin(theta2_ik + theta3_ik);

%THETA4
theta4_ik = atan2(-(c1ik*c23ik*r13 + s1ik*c23ik*r23 - s23ik*r33),-c1ik*s23ik*r13 - s1ik*s23ik*r23 - c23ik*r33);

%THETA5
theta5_ik = atan2(c1ik*r21 - s1ik*r11,c1ik*r22 - s1ik*r12) + pi/2;


q(1) = theta1_ik;
q(2) = theta2_ik;
q(3) = theta3_ik;
q(4) = theta4_ik;
q(5) = theta5_ik;

end

