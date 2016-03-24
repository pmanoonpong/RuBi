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
bendPhase = 2/3;
%toolPoseInit = Initial position of the toes [toolX,toolY, toolTheta]
toolX = 0;
toolY = Ltotal*(bendPhase)*cos(t);
toolTheta = -pi/3*cos(t);
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
m=[0.25;0.20;0.15];
%Y axis displacement
hDelta = 0.1;
%External torque applied to the toes
TauExt=0;
%Moments of inertia of the link (from 1 to 3)
I=[0.001;0.001;0.001];

%IMPULSE ANALYSIS
[FminY, tmax]=impulse(hDelta, m, dispCoG, g);
%External force applied on the toes
fext=[0; -FminY];
disp('Impulse force:');
fext
disp('Time of application:');
tmax

%NEWTON-EULER ALGORITHM FOR DYNAMIC MODEL CONSTRUCTION
%SOLUTION OF THE MODEL FOR t=0
NewtonEulerAnalysis

%ANGULAR DISPLACEMENTS AND VELOCITIES OF THE JOINTS
disp('Initial angular position of joint 1 (degrees):');
vpa((subs(Q1, 0))*360/(2*pi), 6)
disp('Initial angular position of joint 2 (degrees):');
vpa((subs(Q2, 0))*360/(2*pi), 6)
disp('Initial angular position of joint 3 (degrees):');
vpa((subs(Q3, 0))*360/(2*pi), 6)

disp('--------------------------------------');
disp('Angular speed of hip motor (rpm):');
vpa((abs(subs(Q1, 0))/tmax)/(2*pi)*60, 6)
disp('Angular speed of knee motor (rpm):');
vpa((abs(subs(Q2, 0))/tmax)/(2*pi)*60, 6)
disp('Angular speed of ankle motor (rpm):');
vpa((abs(subs(Q3, 0))/tmax)/(2*pi)*60, 6)










