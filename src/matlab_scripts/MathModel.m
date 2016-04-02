%MATHEMATICAL MODEL OF THE BIPEDAL ROBOT
%COMPUTES THE KINEMATICS AND DYNAMICS MODEL OF THE PLATFORM
%PERFORMS THE IMPULSE ANALYSIS FOR THE VERTICAL JUMP CASE 
%OUTPUTS THE NECESSARY TORQUES IN ONE LEG FOR THE DESIRED JUMP 

clear; clc; close all;

%KINEMATICS INPUTS FOR THE MODEL
syms t
%Total length of the leg completely streched
Ltotal = 0.57;
L=[Ltotal*0.46;Ltotal*0.3729;Ltotal*0.1670];
%Percentage of the Ltotal that the leg bends before jumping
bendPhase = 8/13;
%toolPoseInit = Initial position of the toes [toolX,toolY, toolTheta]
toolX = 0;
%Leg bent before jumping
toolY = Ltotal*(bendPhase)*cos(t);
toolTheta = -pi/2.3*cos(t);
%Leg stretched after push-off on running
%toolY = 0.56;
%toolTheta = -pi/8;
%Leg bent before push-off on running
%toolY = 0.4275;
%toolTheta = -pi/2;
%Distance from the joint to the motor position (percentage of link length)
motorPos=2/4;
%Displacement of the CoG during impulse phase
dispCoG = Ltotal - Ltotal*(bendPhase);

%DIRECT AND INVERSE KINEMATICS MODELS
%ANALYSIS OF POSITIONS, VELOCITIES AND ACCELERATIONS IN CARTESIAN AND JOINT
%SPACE FOR ALL THE JOINTS
Kinematics;

%DYNAMICS INPUTS: 
%Gravity (positive in Y axis)
g= 9.81;
%Masses of the links (from 1 to 3)
m=[0.163;0.17;0.029];
%Y axis displacement
hDelta = 0.14;
%External torque applied to the toes
TauExt=0;
%Moments of inertia of the link (from 1 to 3)
I=[0.0001;0.0001;0.0001];

%IMPULSE ANALYSIS
[FminY, tmax, F, t]=impulse(hDelta, m, dispCoG, g);
%External force applied on the toes

for i=1:30
fext=[0; -F(1,i)];
% disp('Impulse force:');
% fext
% disp('Time of application:');
% tmax

%NEWTON-EULER ALGORITHM FOR DYNAMIC MODEL CONSTRUCTION
%SOLUTION OF THE MODEL FOR t=0
NewtonEulerAnalysis

% disp('--------------------------------------');
% disp('Torque in link 1:');
Tau(i,1)=vpa(subs(Torque(1,1),0), 6);
%disp('Torque in link 2:');
Tau(i,2)=vpa(subs(Torque(1,2),0), 6);
%disp('Torque in link 3:');
Tau(i,3)=vpa(subs(Torque(1,3),0), 6);
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
W(i,1)=vpa((abs(subs(Q1, 0))/t(1,i))/(2*pi)*60, 6);
%disp('Angular speed of knee motor (rpm):');
W(i,2)=vpa((abs(subs(Q2, 0))/t(1,i))/(2*pi)*60, 6);
%disp('Angular speed of ankle motor (rpm):');
W(i,3)=vpa((abs(subs(Q3, 0))/t(1,i))/(2*pi)*60, 6);
end

%%
%Peak power calculations
wKnee = vpa((abs(subs(Q2, 0))/tmax), 3);
wAnkle = vpa((abs(subs(Q3, 0))/tmax), 3);
whip = vpa((abs(subs(Q1, 0))/tmax), 3);
PowerHip = vpa(subs(Torque(1,1),0), 3)*whip
PowerKnee = vpa(subs(Torque(1,2),0), 3)* wKnee
PowerAnkle = vpa(subs(Torque(1,3),0), 3)*wAnkle







