%MATHEMATICAL MODEL OF THE BIPEDAL ROBOT
%COMPUTES THE KINEMATICS AND DYNAMICS MODEL OF THE PLATFORM
%PERFORMS THE IMPULSE ANALYSIS FOR THE VERTICAL JUMP CASE 
%OUTPUTS THE NECESSARY TORQUES IN ONE LEG FOR THE DESIRED JUMP 

clear; clc; close all;

%DYNAMICS INPUTS: 
%Gravity (positive in Y axis)
g= 9.81;
%Masses of the links (from 1 to 3)
m=[0.163;0.17;0.029];
%Y axis displacement
hDelta = 0.08;
%External torque applied to the toes
TauExt=0;
%Moments of inertia of the link (from 1 to 3)
I=[0.0001;0.0001;0.0001];

%Total length of the leg completely streched
Ltotal = 0.57;
L=[Ltotal*0.46;Ltotal*0.3729;Ltotal*0.1670];
%Percentage of the Ltotal that the leg bends before jumping
bendPhase = 8/13; %***
%Displacement of the CoG during impulse phase
dispCoG = Ltotal - Ltotal*(bendPhase);

%IMPULSE ANALYSIS
[FminY, tmax, F, t]=impulse(hDelta, m, dispCoG, g);
%%

%External force applied on the toes
i=35;
fext=[0; -F(1,i)];

fext=[0; -FminY];

%KINEMATICS INPUTS FOR THE MODEL
%syms t

%toolPoseInit = Initial position of the toes [toolX,toolY, toolTheta]
toolX = 0;
to = 0;
tf = tmax;

%Leg before jumping and after jumping
toolYf = 0.5481;
toolYo = Ltotal*(bendPhase)
toolThetao=-pi/2;
toolThetaf= -pi/5;

figure
hold on
j=0;
for t=0:0.025:tf
    j=j+1;
    toolY(j) = toolYo + ((toolYf - toolYo)/(tf-to))*(t - to);
    %plot(t, toolY(j), '+');
    toolTheta(j) = toolThetao + ((toolThetaf - toolThetao)/(tf-to))*(t - to);
    %plot(t, toolTheta(j), '+');
    
    
    X2=toolX-L(3,1)*sin(toolTheta(j));
    Y2=toolY(j)-L(3,1)*cos(toolTheta(j));
    Q1= atan(X2/Y2)-acos((X2^2+Y2^2+(L(1,1)^2)-(L(2,1)^2))/(2*L(1,1)*sqrt(X2^2+Y2^2)));
    Q2= pi-acos(((L(1,1)^2)+(L(2,1)^2)-(X2^2)-(Y2^2))/(2*L(1,1)*L(2,1)));
    Q3= toolTheta(j)-Q1-Q2;

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
    
    axis([-0.3 0.3 -0.6 0])
    x1=[vpa(subs(P1(1,1), 0)),vpa(subs(P2(1,1),0))];
    y1=-[vpa(subs(P1(2,1), 0)),vpa(subs(P2(2,1),0))];
    plot(x1,y1)
    x2=[vpa(subs(P2(1,1), 0)),vpa(subs(P3(1,1),0))];
    y2=-[vpa(subs(P2(2,1), 0)),vpa(subs(P3(2,1),0))];
    plot(x2,y2)
    x3=[vpa(subs(P3(1,1), 0)),vpa(subs(P4(1,1),0))];
    y3=-[vpa(subs(P3(2,1), 0)),vpa(subs(P4(2,1),0))];
    plot(x3,y3)
    
end 
hold off

%%


%Leg stretched after push-off on running
%toolY = 0.56;
%toolTheta = -pi/8;
%Leg bent before push-off on running
%toolY = 0.4275;
%toolTheta = -pi/2;
%Distance from the joint to the motor position (percentage of link length)
motorPos=2/4;

%DIRECT AND INVERSE KINEMATICS MODELS
%ANALYSIS OF POSITIONS, VELOCITIES AND ACCELERATIONS IN CARTESIAN AND JOINT
%SPACE FOR ALL THE JOINTS
sysm t;
Kinematics;

%for i=1:round(tmax,2)*100
figure
hold on
for ti=0:0.025:0.25

% disp('Impulse force:');
% fext
% disp('Time of application:');
% tmax

%NEWTON-EULER ALGORITHM FOR DYNAMIC MODEL CONSTRUCTION
%SOLUTION OF THE MODEL FOR t=0
NewtonEulerAnalysis

% disp('--------------------------------------');
% disp('Torque in link 1:');
plot(ti,vpa(subs(Torque(1,1),ti), 3),'r*');
%disp('Torque in link 2:');
plot(ti,vpa(subs(Torque(1,2),ti), 3),'g*');
%disp('Torque in link 3:');
plot(ti,vpa(subs(Torque(1,3),ti), 3),'b*');
%disp('--------------------------------------');


%ANGULAR DISPLACEMENTS AND VELOCITIES OF THE JOINTS
% disp('Initial angular position of joint 1 (degrees):');
% vpa((subs(Q1, 0))*360/(2*pi), 6)
% disp('Initial angular position of joint 2 (degrees):');
% vpa((subs(Q2, 0))*360/(2*pi), 6)
% disp('Initial angular position of joint 3 (degrees):');
% vpa((subs(Q3, 0))*360/(2*pi), 6)

%disp('--------------------------------------');
%disp('Angular speed of hip motor (rpm):');
% W(i,1)=vpa((abs(subs(Q1, 0))/t(1,i))*60/(2*pi), 6);
% %disp('Angular speed of knee motor (rpm):');
% W(i,2)=vpa((abs(subs(Q2, 0))/t(1,i))*60/(2*pi), 6);
% %disp('Angular speed of ankle motor (rpm):');
% W(i,3)=vpa((abs(subs(Q3, 0))/t(1,i))*60/(2*pi), 6);
end
hold off 