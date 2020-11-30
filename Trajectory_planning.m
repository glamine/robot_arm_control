function [a] = Trajectory_planning(x0,xdot0,xf,xdotf,t0,tf)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%    t = linspace(t0,tf,100);
%     px = a1(1) + a2(2).*(t-t0) + a3(3).*(t-t0).^2 + a4(4).*(t-t0).^3;
%     py = ay(1) + ay(2).*(t-t0) + ay(3).*(t-t0).^2 + ay(4).*(t-t0).^3;
%     pz = az(1) + az(2).*(t-t0) + az(3).*(t-t0).^2 + az(4).*(t-t0).^3;

    h = xf - x0;
    T = tf - t0;

    a1 = x0;
    a2 = xdot0;
    a3 = (3*h - (2*xdot0 + xdotf)*T)/(T*T);
    a4 = (-2*h + (xdot0 + xdotf)*T)/(T*T*T);
    
    a = [a1 a2 a3 a4];

end

