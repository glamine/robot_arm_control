scale = 1/10*1/3;%1/20;

d1 = 475*scale; % length of first arm
a1 = 170*scale; % length of second arm
a2 = 600*scale; % length of third arm
a3 = 805*scale; % length of fourth arm
d5 = 300*scale;

%%
N = 20;
% theta1 = linspace(0, pi, N);
% theta2 = linspace(-pi, 0, N);
% theta3 = linspace(0, pi, N);
% theta4 = linspace(-pi/2, pi/2, N);
% theta5 = linspace(0, pi/2, N);


%theta1 = 0:0.1:pi; % all possible theta1 values
%theta2 = -pi:0.1:0; % all possible theta2 values
%theta3 = 0:0.1:pi; % all possible theta3 values
%theta4 = -pi/2:0.1:pi/2; % all possible theta4 values
%theta5 = 0:0.1:pi/2; % all possible theta5 values

%X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2) + l3 * cos(THETA1 + THETA2 + THETA3) + l4 * cos(THETA1 + THETA2 + THETA3 + THETA4) + l5 * cos(THETA1 + THETA2 + THETA3 + THETA4 + THETA5); % compute x coordinates
%Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2) + l3 * sin(THETA1 + THETA2 + THETA3) + l4 * sin(THETA1 + THETA2 + THETA3 + THETA4) + l5 * sin(THETA1 + THETA2 + THETA3 + THETA4 + THETA5); % compute y coordinates
%[THETA1,THETA2,THETA3,THETA4,THETA5] = ndgrid(theta1,theta2,theta3,theta4,theta5);

X = zeros(1,N^4);
Y = zeros(1,N^4);
Z = zeros(1,N^4);

Xc = zeros(1,N^4);
Yc = zeros(1,N^4);
Zc = zeros(1,N^4);
%for i1 = 1 : N
    %theta1 = i1*pi/N;
    theta1 = 0;

    for i2 = 1:N
        theta2 = -pi + i2*pi/N;
        %theta2 = 0;%-pi/2;
        
        for i3 = 1:N
            theta3 = i3*pi/N;
            %theta3 = 0;%pi/2;
            
            for i4 = 1:N
                theta4 = -pi/2 + i4*pi/N; %- pi/2;
                %theta4 = 0;
            
                %for i5 = 1:N
                %    theta5 = ;
                    
                    %indice = i4 + N*(i3-1) + N*N*(i2-1) + N*N*N*(i1-1)
                    %indice = i3 + N*(i2-1) + N*N*(i1-1)
                    %indice = i3 + N*(i2-1)
                    indice = i4 + N*(i3-1) + N*N*(i2-1)
                    
                    X(indice) = -d5*cos(theta1)*sin(theta2 + theta3 + theta4) + a3*cos(theta1)*cos(theta2 + theta3) + (a2*cos(theta2) + a1)*cos(theta1);
                    Y(indice) = -d5*sin(theta1)*sin(theta2 + theta3 + theta4) + a3*sin(theta1)*cos(theta2 + theta3) + (a2*cos(theta2) + a1)*sin(theta1);
                    Z(indice) = -d5*cos(theta2 + theta3 + theta4) - a3*sin(theta2 + theta3) - a2*sin(theta2) + d1;
                    
                    Xc(indice) = a3*cos(theta1)*cos(theta2 + theta3) + (a2*cos(theta2) + a1)*cos(theta1);
                    Yc(indice) = a3*sin(theta1)*cos(theta2 + theta3) + (a2*cos(theta2) + a1)*sin(theta1);
                    Zc(indice) = -a3*sin(theta2 + theta3) - a2*sin(theta2) + d1;

                %end
            end
            
        end
        
    end
    
%end


%data1 = [X(:) Y(:) THETA1(:)]; % create x-y-theta1 dataset
%data2 = [X(:) Y(:) THETA2(:)]; % create x-y-theta2 dataset
%data3 = [X(:) Y(:) THETA3(:)]; % create x-y-theta3 dataset
%data4 = [X(:) Y(:) THETA4(:)]; % create x-y-theta4 dataset
%data5 = [X(:) Y(:) THETA5(:)]; % create x-y-theta5 dataset
%%

close all;
figure(1);
%plot(X(:),Z(:),'r.');
plot(Xc(:),Zc(:),'r.');
  %axis([0 100 0 100]);
  xlabel('X0 [cm]','fontsize',10)
  ylabel('Z0 [cm]','fontsize',10)
  hold on
  %rectangle('Position',[50 0 7.5 27],'EdgeColor','k','LineWidth',2)
  rectangle('Position',[30 0 7.5 27],'EdgeColor','k','LineWidth',2)
  
 % d1
 
 %% inverse kinematics
 
 % O is given
 % R is given
 
%X = -d5*cos(theta1)*sin(theta2 + theta3 + theta4) + a3*cos(theta1)*cos(theta2 + theta3) + (a2*cos(theta2) + a1)*cos(theta1);
%Y = -d5*sin(theta1)*sin(theta2 + theta3 + theta4) + a3*sin(theta1)*cos(theta2 + theta3) + (a2*cos(theta2) + a1)*sin(theta1);
%Z = -d5*cos(theta2 + theta3 + theta4) - a3*sin(theta2 + theta3) - a2*sin(theta2) + d1;
 
theta1 = 0;
theta2 = -pi/2;
theta3 = pi/2;
theta4 = pi;
theta5 = -pi;

c1 = cos(theta1);
s1 = sin(theta1);
c2 = cos(theta2);
s2 = sin(theta2);
c23 = cos(theta2 + theta3);
s23 = sin(theta2 + theta3);
c234 = cos(theta2 + theta3 + theta4);
s234 = sin(theta2 + theta3 + theta4);
c5 = cos(theta5);
s5 = sin(theta5);

x = -d5*c1*s234 + a3*c1*c23 + (a2*c2 + a1)*c1;
y = -d5*s1*s234 + a3*s1*c23 + (a2*c2 + a1)*s1;
z = -d5*c234 - a3*s23 - a2*s2 + d1;

%GOOD

r11 = c1*c234*c5 + s1*s5;
r12 = -c1*c234*s5 + s1*c5; 
r13 = -c1*s234;
r21 = s1*c234*c5 - c1*s5;
r22 = -s1*c234*s5 - c1*c5;
r23 = -s1*s234;
r31 = -s234*c5;
r32 = s234*s5;
r33 = -c234;
%GOOD

%given :
O = [x;y;z];
R = [r11 r12 r13;r21 r22 r23;r31 r32 r33];

%8 vertices check : 

%R always perpen to ground : 
r11 = 1;
r12 = 0; 
r13 = 0;
r21 = 0;
r22 = -1;
r23 = 0;
r31 = 0;
r32 = 0;
r33 = -1;
R = [r11 r12 r13;r21 r22 r23;r31 r32 r33];

% O : position of the 8 vertices,
% (x,y,z) = (30,-7.5/2,0),(30,7.5/2,0),(30+7.5,-7.5/2,0),(30+7.5,7.5/2,0),(30,-7.5/2,27),(30,7.5/2,27),(37.5,-7.5/2,27),(37.5,7.5/2,27);
%30 0 7.5 27

V = [30 30 37.5 37.5 30 30 37.5 37.5; 
     -7.5/2 7.5/2 -7.5/2 7.5/2 -7.5/2 7.5/2 -7.5/2 7.5/2;
     0 0 0 0 27 27 27 27];

n = 4 
O = V(:,n)
 
Oc = O - d5.*R*[0;0;1];

xc = Oc(1);
yc = Oc(2);
zc = Oc(3);
%GOOD

%Solution :

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

theta1_ik
theta2_ik
theta3_ik
theta4_ik
theta5_ik
