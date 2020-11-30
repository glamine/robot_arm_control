%Main script Robot Arm Control

initSerial('COM3',9600);

pause(5) 

% 1)ROBOT PARAMETERS : 
a1 = 0;
a2 = 0.2;
a3 = 0.2;

d1 = 0.2;%adapt au sol
d5 = 0.07;%dépend pince

parameters = [a1 a2 a3 d1 d5];

% 2)GOAL RECEPTION : each new trajectory

%Test 1 goal fixed: 

     qfinal(1) = pi/2;
     qfinal(2) = -pi/2;
     qfinal(3) = pi/2;
     qfinal(4) = 0;
     qfinal(5) = 0;
     
T50final = FK(qfinal,parameters);

xf = T50final(1:3,4);
Rfinal = T50final(1:3,1:3);
xdotf = [0;0;0];
xdot0 = [0;0;0];
 
t0 = 0;
tf = 30;

% 3)look where we are : FK + read sensors
mydata = [0 0 0];%[0 -pi/2 pi/2];
mydata = readData();

%Need to map the int data from arduino to radian value

radianData = IntToRadians(mydata);

%qmes = READ FROM ARDUINO;
% (strcat double2str + use {} and ,)

%test : 
qmes(1) = radianData(1)-pi/2;%0;
qmes(2) = -pi/2;%radianData(2);
qmes(3) = pi/2;%radianData(3);%

%q4 & q5 never read
%need partial IK to find q4 & q5: need R hypothesis
%avoid initial position error, not sure needed.
qmes(4) = 0;%pointing downward
qmes(5) = 0;%never know at beginning 
% ATTENTION CHOOSE it, to fiw the right trajectory
%

T50init = FK(qmes,parameters);
Rinit = T50init(1:3,1:3);
x0 = T50init(1:3,4);

% 4)TRAJECTORY PLANNING :
ax = Trajectory_planning(x0,xdot0,xf,xdotf,t0,tf);
[k,atheta] = Trajectory_PlanningRot(Rinit,Rfinal,t0,tf);

% 5)Loop at each time step, with condition to finish trajectory.
%LOOP 1
epsilon = 0.005;%5 mm
delta = pi/50;%radians

ErrorTotal = Compute_error(xf,x0,Rfinal,Rinit);
distToGoal = norm(ErrorTotal(1:3));
angleToGoal = norm(ErrorTotal(4:6));

% 6)LAUNCH time counter for trajectory
TrajectoryTimer = tic;
StartTimeTrajectory = toc(TrajectoryTimer);

i = 0;
CurrentTime = 0;
n = 3000;
DATA = zeros(11,n);
DataCommand1 = zeros(5,n);
DataCommand2 = zeros(5,n);
TIME = zeros(n);

while (CurrentTime < tf)%(distToGoal > epsilon || angleToGoal > delta) && (CurrentTime < tf)

% 7)REFERENCE GENERATION : each time step
%need Rref, omegaref, xref, xdotref

CurrentTime = toc(TrajectoryTimer);
%[Rref,xref,omegaref,xdotref] = ComputeReference(atheta,ax,k,CurrentTime,StartTimeTrajectory); %need t and t0

%Record data test
 
% DATA(1,floor(i/10)+1) = xref(1);
% DATA(2,floor(i/10)+1) = xref(2);
% DATA(3,floor(i/10)+1) = xref(3);
% 
% DATA(4,floor(i/10)+1) = omegaref(1);
% DATA(5,floor(i/10)+1) = omegaref(2);
% DATA(6,floor(i/10)+1) = omegaref(3);
% 
% DATA(7,floor(i/10)+1) = xdotref(1);
% DATA(8,floor(i/10)+1) = xdotref(2);
% DATA(9,floor(i/10)+1) = xdotref(3);
% 
% TIME(floor(i/10)+1) = CurrentTime;

%Test : constant reference

    qwanted(1) = 0;
    qwanted(2) = -pi/2;
    qwanted(3) = pi/2;
    qwanted(4) = 0;
    qwanted(5) = 0;
    
    T50wanted = FK(qwanted,parameters);

 xref = T50wanted(1:3,4);
 Rref = T50wanted(1:3,1:3);
 xdotref = [0;0;0];
 omegaref = [0;0;0]; %composante omegaref(1) = 0 tjs

% 8)INVERSE KINEMATICS needed for qref for invJac & Rot 0to1

qref = IK(xref,Rref,parameters)

% 9)READ ARDUINO + FK, KNOW WHERE WE ARE :

mydata = readData()

radianData = IntToRadians(mydata)
%Test reading
%qmes = qwanted + [0 0 0 0 0];

qmes(1) = radianData(1)-pi/2%0;
qmes(2) = -pi/2;%radianData(2);%
qmes(3) = pi/2;%radianData(3);%

%qmes = READ FROM ARDUINO;

%not measured
 qmes(4) = qref(4);
 qmes(5) = qref(5);

T50 = FK(qmes,parameters);
R = T50(1:3,1:3);
x = T50(1:3,4);

% 10)ERROR COMPUTATION : 
ec = Compute_error(xref,x,Rref,R); %x and R from FK + measures

% 11)P CONTROLLER
xdotc0 = PIController(ec,xdotref,omegaref);

% 12)FROM 0 TO 1 FRAME need IK
R01 = [  cos(qref(1)), sin(qref(1)),  0;
                    0,            0, -1;
        -sin(qref(1)), cos(qref(1)),  0];
    
xdotc1(1:3) = R01*xdotc0(1:3);
xdotc1(4:6) = R01*xdotc0(4:6);    

% 13)FROM END EFFECTOR TO JOINTS
qdotcommand1 = InvJacobian5by5(xdotc1',qmes,parameters); %need IK %qref ou qmes?
qdotcommand2 = InvJacobian5by5(xdotc1',qref,parameters)

%Collect Data :
DataCommand1(:,floor(i/10)+1) = qdotcommand1;
DataCommand2(:,floor(i/10)+1) = qdotcommand2;

% 14)SEND COMMAND TO ARDUINO:

%Need map commands to int value for arduino

intSpeed = mapSpeed(qdotcommand2(1:3));
intSpeed(4) = 1;%q4
intSpeed(5) = 1;%clamp
intSpeed
%intSpeed = [qdotcommand2(1) qdotcommand2(2) qdotcommand2(3) 1 1]; % 1 or 0 for 5 and clamp, time is not going to be easy
transmitData(intSpeed);

% 15)Compute error to goal :

ErrorTotal = Compute_error(xf,x,Rfinal,R);
distToGoal = norm(ErrorTotal(1:3));
angleToGoal = norm(ErrorTotal(4:6));

i = i + 1;

end %end LOOP1

endSerial();

%% PLOTS tests

p = 2878;

% figure(1);
% plot(TIME(1:p),DATA(1,1:p)); 
% title('x position')
% xlabel('Temps [s]')
% ylabel('Position [m]')
% 
% figure(2);
% plot(TIME(1:p),DATA(2,1:p)); 
% title('y position')
% xlabel('Temps [s]')
% ylabel('Position [m]')
% 
% figure(3);
% plot(TIME(1:p),DATA(3,1:p)); 
% title('z position')
% xlabel('Temps [s]')
% ylabel('Position [m]')
% 
% figure(4);
% plot(TIME(1:p),DATA(7,1:p)); 
% title('x omega')
% xlabel('Temps [s]')
% ylabel('Position [m]')
% 
% figure(5);
% plot(TIME(1:p),DATA(8,1:p)); 
% title('y omega')
% xlabel('Temps [s]')
% ylabel('Position [m]')
% 
% figure(6);
% plot(TIME(1:p),DATA(9,1:p)); 
% title('z omega')
% xlabel('Temps [s]')
% ylabel('Position [m]')

% figure(7);
% plot(TIME(1:p),DataCommand1(1,1:p)); 
% title('qdotc 1')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
% 
% figure(8);
% plot(TIME(1:p),DataCommand1(2,1:p)); 
% title('qdotc 2')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
% 
% figure(9);
% plot(TIME(1:p),DataCommand1(3,1:p)); 
% title('qdotc 3')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
% 
% figure(10);
% plot(TIME(1:p),DataCommand1(4,1:p)); 
% title('qdotc 4')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
% 
% figure(11);
% plot(TIME(1:p),DataCommand1(5,1:p)); 
% title('qdotc 5')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')

%

% figure(12);
% plot(TIME(1:p),DataCommand2(1,1:p)); 
% title('qdotc ref 1')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
% 
% figure(13);
% plot(TIME(1:p),DataCommand2(2,1:p)); 
% title('qdotc ref 2')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
% 
% figure(14);
% plot(TIME(1:p),DataCommand2(3,1:p)); 
% title('qdotc ref 3')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
% 
% figure(15);
% plot(TIME(1:p),DataCommand2(4,1:p)); 
% title('qdotc ref 4')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
% 
% figure(16);
% plot(TIME(1:p),DataCommand2(5,1:p)); 
% title('qdotc ref 5')
% xlabel('Temps [s]')
% ylabel('thetadot [rad/s]')
