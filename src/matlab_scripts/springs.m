%%PARALLEL SPRINGS

clc; clear; close all;

%_______________________________________
%Stand-upstraight phase (two legs, no movement)
%Calculus for k2 based on position of com
Ltotal = 0.57;
L=[Ltotal*0.46;Ltotal*0.3729;Ltotal*0.1670];
mTotal= 0.800;
g = 9.81;
%Angular change in the springs during normal push-off
deltaQ3 = pi/2;
deltaQ2 = pi/4;

%Total torque on the ankles produced by the body weight
dAnkleCOM=[0.03;0.28];                              %Distance of the com to the ankle (in same plane) 
Fg = [0;mTotal*g];
alpha = acos(dAnkleCOM(2,1)/norm(dAnkleCOM));       %Inclination of d wrt Y axis

FgAnkleX = norm(Fg)*sin(alpha);
torqueBodyOverAnkles = FgAnkleX*norm(dAnkleCOM);
torqueBodyOverAnkle = torqueBodyOverAnkles/2;

K3 = 1.4*torqueBodyOverAnkle/deltaQ3               %This torque is used as a constraint to determine K3

%Total torque on the knees produced by the body weight
dAnkleKnee=[0.092;0.19];  
dKneeCOM = dAnkleCOM - dAnkleKnee;

alpha2 = acos(-dKneeCOM(1,1)/norm(dKneeCOM));
FgKneeX = norm(Fg)*cos(alpha2);
torqueBodyOverKnees = FgKneeX*norm(dKneeCOM);

%_______________________________________
%Midstance phase (one leg): Energy storage
deltaLeq = 0.13;
mTotal= 0.971;
Froud = 0.2;
Vcom = sqrt(Froud*(L(1,1)+L(2,1))*g);
CteEnergy = 0.6;                            %Percentage of the Kinetic energy storaged during midstance phase
KineticE = (1/2)*mTotal*Vcom^2;
Keq = (KineticE)*(CteEnergy)*2/deltaLeq^2;
K2 = (Keq*deltaLeq^2 - K3*deltaQ3^2)/deltaQ2^2

%For the midstanding phase case (in one leg), the resulting torques are:
torqueSpringKnee = (K2*pi/2)
torqueSpringAnkle = (K3*pi/4)

%Energy storaged in one knee and ankle
kneeEnergy = (1/2)*K2*deltaQ2^2
ankleEnergy = (1/2)*K3*deltaQ3^2

disp('Percentages of the total energy applied by knee and ankle:');
kneeEnergy*100/(kneeEnergy+ankleEnergy)
ankleEnergy*100/(kneeEnergy+ankleEnergy)

disp('Height reachable as a transformation to potential energy:');
deltaH= (KineticE*CteEnergy)/(mTotal*g)

%%
%SERIES SPRINGS

%Knee
legsMass = 0.8;
Kkmax=0.1848*legsMass;
TauKmax=Kkmax*75
Kkmin=0.0873*legsMass;
TauKmin=Kkmin*75

Kamax=0.1239;
TauAmax=Kamax*35
Kamin=0.0887;
TauAmin=Kamin*35














