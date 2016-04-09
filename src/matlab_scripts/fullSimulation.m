%TOES DISPLACEMENT AS A FUNCTION OF TIME 
syms t;
toolY = toolYo + ((toolYf - toolYo)/(tf-to))*(t - to);
toolTheta = toolThetaO + ((toolThetaF - toolThetaO)/(tf-to))*(t - to);
%Distance from the joint to the motor position (percentage of link length)
motorPos=2/4;


%Leg stretched after push-off on running
%toolY = 0.56;
%toolTheta = -pi/8;
%Leg bent before push-off on running
%toolY = 0.4275;
%toolTheta = -pi/2;

%DIRECT AND INVERSE KINEMATICS MODELS
%ANALYSIS OF POSITIONS, VELOCITIES AND ACCELERATIONS IN CARTESIAN AND JOINT
%SPACE FOR ALL THE JOINTS
Kinematics;

%NEWTON-EULER ALGORITHM FOR DYNAMIC MODEL CONSTRUCTION
NewtonEulerAnalysis


figure
hold on
h=0;
for ti=0:0.02:tf
h=h+1;
%SOLVE SYMBOLIC EQUATIONS: 
%TORQUE(t)
% disp('--------------------------------------');
% disp('Torque in link 1:');
Tau(h,1)=vpa(subs(Torque(1,1),ti), 3);
plot(ti,Tau(h,1),'r*');
%disp('Torque in link 2:');
Tau(h,2)=vpa(subs(Torque(1,2),ti), 3);
plot(ti,Tau(h,2),'g*');
%disp('Torque in link 3:');
Tau(h,3)=vpa(subs(Torque(1,3),ti), 3);
plot(ti,Tau(h,3),'b*');
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
W(h,1)=vpa((abs(subs(Q1, 0))/ti)*60/(2*pi), 6);
% %disp('Angular speed of knee motor (rpm):');
W(h,2)=vpa((abs(subs(Q2, 0))/ti)*60/(2*pi), 6);
% %disp('Angular speed of ankle motor (rpm):');
W(h,3)=vpa((abs(subs(Q3, 0))/ti)*60/(2*pi), 6);
end
hold off 

