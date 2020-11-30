function [k,a] = Trajectory_PlanningRot(Rinit,Rfinal,t0,tf)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%     theta(t) = a1 + a2.*(t-t0) + a3.*(t-t0).^2 + a4.*(t-t0).^3;

Rot = (Rinit')*Rfinal;

theta_f = 2*acos((1/2)*sqrt(1 + Rot(1,1) + Rot(2,2) + Rot(3,3))); %retourne que entre 0 et pi!! problème?
theta_0 = 0; %?? yes
thetadot_0 = 0;
thetadot_f = 0;

    h = theta_f - theta_0;
    T = tf - t0;

    a1 = theta_0;
    a2 = thetadot_0;
    a3 = (3*h - (2*thetadot_0 + thetadot_f)*T)/(T*T);
    a4 = (-2*h + (thetadot_0 + thetadot_f)*T)/(T*T*T);
    
    a = [a1 a2 a3 a4];
    
    k = [Rot(3,2) - Rot(2,3);Rot(1,3) - Rot(3,1);Rot(2,1) - Rot(1,2)];
    
    k = 1/(2*sin(theta_f))*k;


end

