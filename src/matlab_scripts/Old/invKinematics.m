function[Q]=invKinematics(toolPose,L)
%Inputs: (in radians so far)
%toolPose= 3x1 Vector containing [X,Y,theta] position of the end-effector
%L = vector contaning the lengths of the links

X2=toolPose(1,1)-L(3,1)*sin(toolPose(3,1));
Y2=toolPose(2,1)-L(3,1)*cos(toolPose(3,1));
Q1= atan(X2/Y2)-acos((X2^2+Y2^2+L(1,1)^2-L(2,1)^2)/(2*L(1,1)*sqrt(X2^2+Y2^2)));
Q2= pi-acos((L(1,1)^2+L(2,1)^2-X2^2-Y2^2)/2*L(1,1)*L(2,1));
Q3= toolPose(3,1)-Q1-Q2;
%dQ = Qf-Qo;
Q = [Q1; Q2; Q3];