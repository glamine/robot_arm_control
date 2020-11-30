function [ec] = Compute_error(xref,x,Rref,R)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

etr = xref - x;

eor = (1/2)*(cross(R(:,1),Rref(:,1)) + cross(R(:,2),Rref(:,2)) + cross(R(:,3),Rref(:,3)));

ec = [etr;eor];


end

