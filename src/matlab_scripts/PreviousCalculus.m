%Clean workspace
clear 
clc

%Obtain joints linear positions, velocities and accelerations
syms Q1 Q2 Q3 
syms L1 L2 L3
L=[L1;L2;L3];
Qsym=[Q1;Q2;Q3];

%Compute cartesian position from forward kinematics model
Px=L(1,1)*sin(Qsym(1,1))+L(2,1)*sin(Qsym(1,1)+Qsym(2,1))+L(3,1)*sin(Qsym(1,1)+Qsym(2,1)+Qsym(3,1));
Py=L(1,1)*cos(Qsym(1,1))+L(2,1)*cos(Qsym(1,1)+Qsym(2,1))+L(3,1)*cos(Qsym(1,1)+Qsym(2,1)+Qsym(3,1));
Ptheta=Qsym(1,1)+Qsym(2,1)+Qsym(3,1);

P=[Px;Py;Ptheta];

%Calculate the linear speed as the derivate of the linear displacement
dP=[diff(P(1,1), Qsym(1,1), Qsym(2,1), Qsym(3,1));
    diff(P(2,1), Qsym(1,1), Qsym(2,1), Qsym(3,1));
    diff(P(3,1), Qsym(1,1), Qsym(2,1), Qsym(3,1))];
    
%Calculate the linear acceleration as the derivate of the linear velocity
ddP=[diff(dP(1,1), Qsym(1,1), Qsym(2,1), Qsym(3,1));
     diff(dP(2,1), Qsym(1,1), Qsym(2,1), Qsym(3,1));
     diff(dP(3,1), Qsym(1,1), Qsym(2,1), Qsym(3,1))];

%Generic jacobian
J = jacobian([P(1,1);P(2,1)], [Qsym(1,1),Qsym(2,1),Qsym(3,1)]);
%P(3,1)



%Obtain joints angular positions, velocities and accelerations
syms toolPoseX toolPoseY toolPoseTheta
syms L1 L2 L3
L=[L1;L2;L3];
toolPose=[toolPoseX;toolPoseY;toolPoseTheta];

%Inverse kinematics model
X2=toolPose(1,1)-L(3,1)*sin(toolPose(3,1));
Y2=toolPose(2,1)-L(3,1)*cos(toolPose(3,1));
Q1= atan(X2/Y2)-acos((X2^2+Y2^2+L(1,1)^2-L(2,1)^2)/(2*L(1,1)*sqrt(X2^2+Y2^2)));
Q2= pi-acos((L(1,1)^2+L(2,1)^2-X2^2-Y2^2)/2*L(1,1)*L(2,1));
Q3= toolPose(3,1)-Q1-Q2;
Q = [Q1; Q2; Q3];

%Calculate the angular speed as the derivate of the angular displacement
dQ=[diff(Q(1,1), toolPose(1,1), toolPose(2,1), toolPose(3,1));
    diff(Q(2,1), toolPose(1,1), toolPose(2,1), toolPose(3,1));
    diff(Q(3,1), toolPose(1,1), toolPose(2,1), toolPose(3,1));];

%Calculate the angular acceleration as the derivate of the angular velocity
ddQ=[diff(dQ(1,1), toolPose(1,1), toolPose(2,1), toolPose(3,1));
     diff(dQ(2,1), toolPose(1,1), toolPose(2,1), toolPose(3,1));
     diff(dQ(3,1), toolPose(1,1), toolPose(2,1), toolPose(3,1));];



%Numerical solution for the equations obtained: introduce kinematics model
%parameters here
threshold = 0.0001;
L1=1;
L2=1;
L3=1;
L=[L1;L2;L3];
%Toes initial position (X=0)
PinitialY=2;
PinitialTheta=pi/4;
%Toes final position (leg streched on Y axis)
PfinalY=L1+L2+L3;
PfinalTheta=0;

toolPoseInit=[0;
              PinitialY;
              PinitialTheta];
          
toolPoseFinal=[0;
               PfinalY;
               PfinalTheta];
           
%Initial angular position
toolPoseX=toolPoseInit(1,1);
toolPoseY=toolPoseInit(2,1);
toolPoseTheta=toolPoseInit(3,1);
Qo=vpa(subs(Q));

%Final angular position
toolPoseX=toolPoseFinal(1,1);
toolPoseY=toolPoseFinal(2,1);
toolPoseTheta=toolPoseFinal(3,1);
Qf=vpa(subs(Q));

%Angular displacement:
dispQ=Qf-Qo

%Linear displacement of the end-effector
toolPoseX=toolPoseFinal(1,1)-toolPoseInit(1,1);
toolPoseY=toolPoseFinal(2,1)-toolPoseInit(2,1);
toolPoseTheta=toolPoseFinal(3,1)-toolPoseInit(3,1);

%Angular velocity
velQ=vpa(subs(dQ))
%Angular acceleration
accQ=vpa(subs(ddQ))

%Obtain the jacobian J(q) for q=Qo 
Q1=Qo(1,1);
Q2=Qo(2,1);
Q3=Qo(3,1);
Jq=subs(J);

%Linear displacement
Po=vpa(subs(P));
ddispP=vpa(subs(dP));
Q1=Qf(1,1);
Q2=Qf(2,1);
Q3=Qf(3,1);
Pf=vpa(subs(P));

dispP=Pf-Po;
dispP(dispP < threshold) = 0;
%display(dispP, 'Linear displacement');

