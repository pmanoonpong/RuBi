%MATHEMATICAL MODEL OF THE BIPEDAL ROBOT
%COMPUTES THE KINEMATICS AND DYNAMICS MODEL OF THE PLATFORM
%PERFORMS THE IMPULSE ANALYSIS FOR THE VERTICAL JUMP CASE 
%OUTPUTS THE NECESSARY TORQUES IN ONE LEG FOR THE DESIRED JUMP 

clear; clc;
close all;

%DYNAMICS INPUTS: 
%Gravity (positive in Y axis)
g= [0; 9.81];
%Masses of the links (from 1 to 3)
m=[0.123;0.115;0.021];
%Y axis displacement
hDelta = 0.05;
%External torque applied to the toes
TauExt=0;
%Moments of inertia of the link (from 1 to 3)
I=[0.0001;0.0001;0.0001];

%Total length of the leg completely streched
Ltotal = 0.57;
L=[Ltotal*0.46;Ltotal*0.3729;Ltotal*0.1670];
%Percentage of the Ltotal that the leg bends before jumping
bendPhase = 9/13; 
%Displacement of the CoG during impulse phase
dispCoG = Ltotal - Ltotal*(bendPhase);

%IMPULSE ANALYSIS
[FminY, tmax, F, t]=impulse(hDelta, m, dispCoG);

%KINEMATICS INPUTS FOR THE MODEL
%External force applied on the toes
fext=[0; -FminY];
toolX = 0;
to = 0;
tf = tmax;

%Leg before and after jumping
toolYo = Ltotal*(bendPhase);
toolYf = 0.55;
toolThetaO=-pi/2;
toolThetaF= -pi/8;

figure
xlabel('X axis (m)');
ylabel('Inverted Y axis (m)');
hold on
j=0;
for t=0:0.025:tf
    j=j+1;
    toolY(j) = toolYo + ((toolYf - toolYo)/(tf-to))*(t - to);
    %plot(t, toolY(j), '+');
    toolTheta(j) = toolThetaO + ((toolThetaF - toolThetaO)/(tf-to))*(t - to);
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

    %Plot poses of the leg (inverted for visualization)
%     plot(P1(1,1), P1(2,1));
%     plot(P2(1,1), P2(2,1));
%     plot(P3(1,1), P3(2,1));
%     plot(P4(1,1), P4(2,1));
    
    axis([-0.3 0.3 -0.6 0])
    x1=[vpa(subs(P1(1,1), 0)),vpa(subs(P2(1,1),0))];
    y1=-[vpa(subs(P1(2,1), 0)),vpa(subs(P2(2,1),0))];
    plot(x1,y1, 'Color',[(j/100)*7,(j/100)*7,0])
    x2=[vpa(subs(P2(1,1), 0)),vpa(subs(P3(1,1),0))];
    y2=-[vpa(subs(P2(2,1), 0)),vpa(subs(P3(2,1),0))];
    plot(x2,y2, 'Color',[(j/100)*7,(j/100)*7,0])
    x3=[vpa(subs(P3(1,1), 0)),vpa(subs(P4(1,1),0))];
    y3=-[vpa(subs(P3(2,1), 0)),vpa(subs(P4(2,1),0))];
    plot(x3,y3, 'Color',[(j/100)*7,(j/100)*7,0])
    
end 
hold off
%%
fullSimulation
 