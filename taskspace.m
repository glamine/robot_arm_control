l1 = 30; % length of first arm
l2 = 25; % length of second arm
l3 = 25; % length of third arm
l4 = 10; % length of fourth arm
l5 = 5; % length of fifth arm

theta1 = 0; % all possible theta1 values
theta2 = 0:0.1:pi/2; % all possible theta2 values
theta3 = -pi/2:0.1:0; % all possible theta3 values
theta4 = -pi/2:0.1:0; % all possible theta4 values
theta5 = 0; % all possible theta5 values

[THETA1,THETA2,THETA3,THETA4,THETA5] = ndgrid(theta1,theta2,theta3,theta4,theta5); % generate a grid of theta1 and theta2 values

X = l1 * cos(THETA1) + l2 * cos(THETA1 + THETA2) + l3 * cos(THETA1 + THETA2 + THETA3) + l4 * cos(THETA1 + THETA2 + THETA3 + THETA4) + l5 * cos(THETA1 + THETA2 + THETA3 + THETA4 + THETA5); % compute x coordinates
Y = l1 * sin(THETA1) + l2 * sin(THETA1 + THETA2) + l3 * sin(THETA1 + THETA2 + THETA3) + l4 * sin(THETA1 + THETA2 + THETA3 + THETA4) + l5 * sin(THETA1 + THETA2 + THETA3 + THETA4 + THETA5); % compute y coordinates

data1 = [X(:) Y(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Y(:) THETA2(:)]; % create x-y-theta2 dataset
data3 = [X(:) Y(:) THETA3(:)]; % create x-y-theta3 dataset
data4 = [X(:) Y(:) THETA4(:)]; % create x-y-theta4 dataset
data5 = [X(:) Y(:) THETA5(:)]; % create x-y-theta5 dataset

plot(X(:),Y(:),'r.');
  axis([0 100 0 100]);
  xlabel('centimeters','fontsize',10)
  ylabel('centimeters','fontsize',10)
  hold on
  rectangle('Position',[70 0 7.5 27],'EdgeColor','k','LineWidth',2)