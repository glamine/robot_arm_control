function [speedInt] = mapSpeed(speedRad)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

speedInt(1) = 90 + round(speedRad(1)*(45/pi));%int 0 a 180 NOTE : 180 = 2*pi rad/s
    if(speedInt(1) > 180)
        speedInt(1) = 180;
    elseif(speedInt(1) < 0)
        speedInt(1) = 0;
    end     
speedInt(2) = 90 + round(speedRad(2)*(45/pi));%int 
    if(speedInt(2) > 180)
        speedInt(2) = 180;
    elseif(speedInt(2) < 0)
        speedInt(2) = 0;
    end
speedInt(3) = 90 + round(speedRad(3)*(45/pi));
    if(speedInt(3) > 180)
        speedInt(3) = 180;
    elseif(speedInt(3) < 0)
        speedInt(3) = 0;
    end

end

