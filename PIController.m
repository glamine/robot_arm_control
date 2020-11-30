function [xdotcommand] = PIController(ec,xdotref,omegaref)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%xcdot = [xcdot;omega]

%LETS DO A SIMPLE P CONTROLLER !

%Time = tic;
%NEED FOR INITIALISATION!!

%dt = Time - toc(previousTime); % resoudre probleme reference de temps

errP = ec;

%errI = ErrtotI + ec*dt;
Kplin = 5;
%Kp1 = ;%trans
%Kp2 = ;
%Kp3 = ;

Kprot = 5;
%Kp4 = ;%rot
%Kp5 = ;
%Kp6 = ;

xcdot = xdotref + [Kplin*errP(1); Kplin*errP(2); Kplin*errP(3)]; %+ [Ki1*errI(1); Ki2*errI(2); Ki3*errI(3)];

omega = omegaref + [Kprot*errP(4); Kprot*errP(5); Kprot*errP(6)]; %+ [Ki4*errI(1); Ki5*errI(2); Ki6*errI(3)];

xdotcommand = [xcdot;omega];

%

%ErrtotI = errI;

%previousTime = Time;

end

