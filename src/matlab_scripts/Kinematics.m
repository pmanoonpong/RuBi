%Direct and inverse kinematics models for a kinematic chain of three joints
%The inputs are the positions of the end-effector of the chain in cartesian
%coordinates and the lengths of the links

%Map toes linear movement function to joints space 
X2=toolX-L(3,1)*sin(toolTheta);
Y2=toolY-L(3,1)*cos(toolTheta);
Q1= atan(X2/Y2)-acos((X2^2+Y2^2+(L(1,1)^2)-(L(2,1)^2))/(2*L(1,1)*sqrt(X2^2+Y2^2)));
Q2= pi-acos(((L(1,1)^2)+(L(2,1)^2)-(X2^2)-(Y2^2))/(2*L(1,1)*L(2,1)));
Q3= toolTheta-Q1-Q2;

%Differentiation of the direct kinematics model for linear dynamics

%DK model for joints positions
X0=0;
Y0=0;
X1=L(1,1)*sin(Q1);
Y1=L(1,1)*cos(Q1);
X2=L(1,1)*sin(Q1)+L(2,1)*sin(Q1+Q2);
Y2=L(1,1)*cos(Q1)+L(2,1)*cos(Q1+Q2);
X3=L(1,1)*sin(Q1)+L(2,1)*sin(Q1+Q2)+L(3,1)*sin(Q1+Q2+Q3);
Y3=L(1,1)*cos(Q1)+L(2,1)*cos(Q1+Q2)+L(3,1)*cos(Q1+Q2+Q3);

P1=[X0;Y0];
P2=[X1;Y1];
P3=[X2;Y2];
P4=[X3;Y3];
Positions=[P1,P2,P3,P4];

%Plot position of the leg (inverted for visualization)
% hold on
% axis([-0.3 0.3 -0.6 0])
% x1=[vpa(subs(P1(1,1), 0)),vpa(subs(P2(1,1),0))];
% y1=-[vpa(subs(P1(2,1), 0)),vpa(subs(P2(2,1),0))];
% plot(x1,y1)
% x2=[vpa(subs(P2(1,1), 0)),vpa(subs(P3(1,1),0))];
% y2=-[vpa(subs(P2(2,1), 0)),vpa(subs(P3(2,1),0))];
% plot(x2,y2)
% x3=[vpa(subs(P3(1,1), 0)),vpa(subs(P4(1,1),0))];
% y3=-[vpa(subs(P3(2,1), 0)),vpa(subs(P4(2,1),0))];
% plot(x3,y3)
% hold off

%DK model for positions of centers of mass (assuming they are in the center of
%the links
Xcom0=0;
Ycom0=0;
Xcom1=(L(1,1)*motorPos)*sin(Q1);
Ycom1=(L(1,1)*motorPos)*cos(Q1);
Xcom2=L(1,1)*sin(Q1)+(L(2,1)*motorPos)*sin(Q1+Q2);
Ycom2=L(1,1)*cos(Q1)+(L(2,1)*motorPos)*cos(Q1+Q2);
Xcom3=L(1,1)*sin(Q1)+L(2,1)*sin(Q1+Q2)+(L(3,1)*motorPos)*sin(Q1+Q2+Q3);
Ycom3=L(1,1)*cos(Q1)+L(2,1)*cos(Q1+Q2)+(L(3,1)*motorPos)*cos(Q1+Q2+Q3);

Pcom1=[Xcom0;Ycom0];
Pcom2=[Xcom1;Ycom1];
Pcom3=[Xcom2;Ycom2];
Pcom4=[Xcom3;Ycom3];
PosComs=[Pcom1,Pcom2,Pcom3,Pcom4];

%Linear velocities of the centers of mass
V0=[diff(Pcom1(1,1),t);diff(Pcom1(2,1),t)];
V1=[diff(Pcom2(1,1),t);diff(Pcom2(2,1),t)];
V2=[diff(Pcom3(1,1),t);diff(Pcom3(2,1),t)];
V3=[diff(Pcom4(1,1),t);diff(Pcom4(2,1),t)];

Velocities=[V0,V1,V2,V3];

%Linear accelerations of the centers of mass 
dV0=[diff(V0(1,1),t);diff(V0(2,1),t)];
dV1=[diff(V1(1,1),t);diff(V1(2,1),t)];
dV2=[diff(V2(1,1),t);diff(V2(2,1),t)];
dV3=[diff(V3(1,1),t);diff(V3(2,1),t)];

Accelerations=[dV0,dV1,dV2,dV3];

%Differentiation of the direct kinematics model for angular dynamics
%Joint velocities
dQ1= diff(Q1, t);
dQ2= diff(Q2, t);
dQ3= diff(Q3, t);
%Joint accelerations
ddQ1= diff(dQ1, t);
ddQ2= diff(dQ2, t);
ddQ3= diff(dQ3, t);

%Rewrite joint dynamics as angular dynamics
%Angular velocities
W1 = dQ1;
W2 = W1+dQ2;
W3 = W2+dQ3;
%Angular accelerations
dW1= diff(W1, t);
dW2= diff(W2, t);
dW3= diff(W3, t);
dW=[dW1;dW2;dW3];











