function [Positions]=directKinematics(angularPositions, L)
%Inputs: (in radians so far)
%angularPositions = 3x1 vector containing Qi
%L = vector contaning the lengths of the links

%Direct kinematics model
Q1 = angularPositions(1,1);
Q2 = angularPositions(2,1);
Q3 = angularPositions(3,1);
L1 = L(1,1);
L2 = L(2,1);
L3 = L(3,1);

%Planar kinematic chain with three revolute joints
%Reference frame in hips center
X0=0;
Y0=0;
X1=L1*sin(Q1);
Y1=L1*cos(Q1);
X2=L1*sin(Q1)+L2*sin(Q1+Q2);
Y2=L1*cos(Q1)+L2*cos(Q1+Q2);
Px=L1*sin(Q1)+L2*sin(Q1+Q2)+L3*sin(Q1+Q2+Q3);
Py=L1*cos(Q1)+L2*cos(Q1+Q2)+L3*cos(Q1+Q2+Q3);

%Vectors with the position of every joint
P1=[X0;Y0;0];
P2=[X1;Y1;0];
P3=[X2;Y2;0];
P4=[Px;Py;0];
Positions=[P1,P2,P3,P4];