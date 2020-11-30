% Test communication with arduino

initSerial('COM3',9600);

pause(3) 

%Loop of control for robot.
%handleSerial(btnState);

tf = 3;
TrajectoryTimer = tic;
StartTimeTrajectory = toc(TrajectoryTimer);
CurrentTime = 0;
i = 0;

    mydata = readData()

while (CurrentTime < tf) %&& i < 500
        
    
    mydata = readData()
    %pause(1);

    %q = [1.2 2.3 3.4 4.5 5.6];
    q = [1*i 2*i 3*i 4*i 5*i];
    transmitData(q)
    %pause(1);
    
    
    CurrentTime = toc(TrajectoryTimer);

    i = i+1;
end

endSerial();