%Energy method algorithm for calculus of the torques in static situation
%Currently under development
clc
clear

Lnum=[0.15;0.15;0.05];
toolX = 0;
toolY = 0.2;
toolTheta = pi/4;

X2=toolX-Lnum(3,1)*sin(toolTheta);
Y2=toolY-Lnum(3,1)*cos(toolTheta);
Q1num= atan(X2/Y2)-acos((X2^2+Y2^2+(Lnum(1,1)^2)-(Lnum(2,1)^2))/(2*Lnum(1,1)*sqrt(X2^2+Y2^2)));
Q2num= pi-acos(((Lnum(1,1)^2)+(Lnum(2,1)^2)-(X2^2)-(Y2^2))/(2*Lnum(1,1)*Lnum(2,1)));
Q3num= toolTheta-Q1num-Q2num;

syms Q1 Q2 Q3 L1 L2 L3
L=[L1;L2;L3];
X3=L(1,1)*sin(Q1)+L(2,1)*sin(Q1+Q2)+L(3,1)*sin(Q1+Q2+Q3);
Y3=L(1,1)*cos(Q1)+L(2,1)*cos(Q1+Q2)+L(3,1)*cos(Q1+Q2+Q3);

%X3=Lnum(1,1)*sin(Q1num)+Lnum(2,1)*sin(Q1num+Q2num)+Lnum(3,1)*sin(Q1num+Q2num+Q3num);
%Y3=Lnum(1,1)*cos(Q1num)+Lnum(2,1)*cos(Q1num+Q2num)+Lnum(3,1)*cos(Q1num+Q2num+Q3num);

J=jacobian([X3,Y3],[Q1,Q2,Q3]);
Jinv=pinv(J);
g=9.81;
m=[0.15;0.15;0.15];
fext=[0; -(m(1,1)+m(2,1)+m(3,1))*g];
Torque=Jinv*fext;
vpa(subs(Torque, [Q1,Q2,Q3,L1,L2,L3], [Q1num, Q2num, Q3num, Lnum(1,1), Lnum(2,1), Lnum(3,1)]))
