%function[TORQUE] = NewtonEulerAlgorithm2 (n, Fext, Next, L, m)
%Inputs: 
%n = number of joints 
%L[Li] = Vector with the lenghts of the links
%m[mi] = Vector with the masses of the links
%Fext = External force applied to the K. chain
%Next = External torque applied to the K. chain

%DYNAMICS PARAMETERS
%SYMBOLIC INPUTS FOR TESTING
% n=3;                            %Number of joints in the chain
% syms g;
% syms m1 m2 m3
% syms FextX FextY NextX NextY 
% m=[m1;m2;m3];
% fext=[FextX; FextY];
% TauExt=0;
% % dQ=[dQ1;dQ2;dQ3];
% % ddQ=[ddQ1;ddQ3;ddQ3];
% % Q=[Q1;Q2;Q3];
% syms r;clc
% syms I;
% I=[I;I;I];

clc

%DYNAMICS PARAMETERS: real values
n=3;
r=1;
g=9.81;
m=[0.2;0.08;0.01];
fext=[0; -(m(1,1)+m(2,1)+m(3,1))*g];
TauExt=0;
I=[0.02;0.02;0.02];

% %INPUTS FROM KINEMATICS ANALYSIS
% Q=[Qo(1,1);Qo(2,1);Qo(3,1)];
% dQ=[velQ(1,1);velQ(2,1);velQ(3,1)];
% ddQ=[accQ(1,1);accQ(2,1);accQ(3,1)];

%N-E FORWARD STEP

%Link i=3
f23 = fext - m(3,1)*g + m(3,1)*Accelerations(:,4);

r2c3 = (Positions(:,4)-Positions(:,3))/2;
r3c3 = (Positions(:,3)-Positions(:,4))/2;

Torque(1,3) = TauExt + [r2c3(1,1)*f23(2,1)-r2c3(2,1)*f23(1,1)] - [r3c3(1,1)*fext(2,1)-r3c3(2,1)*fext(1,1)] + I(3,1)*dW(3,1);

%Link i=2
f12 = f23 - m(2,1)*g + m(2,1)*Accelerations(:,3);

r1c2 = (Positions(:,3)-Positions(:,2))/2;
r2c2 = (Positions(:,2)-Positions(:,3))/2;

Torque(1,2) = Torque(1,3) + [r1c2(1,1)*f12(2,1)-r1c2(2,1)*f12(1,1)] - [r2c2(1,1)*f23(2,1)-r2c2(2,1)*f23(1,1)] + I(2,1)*dW(2,1);

%Link i=1
f01 = f12 -m(1,1)*g + m(1,1)*Accelerations(:,2);

r0c1 = Positions(:,2)/2;
r1c1 = (Positions(:,2)-Positions(:,1))/2;

Torque(1,1) = Torque(1,2) + [r0c1(1,1)*f01(2,1)-r0c1(2,1)*f01(1,1)] - [r1c1(1,1)*f12(2,1)-r1c1(2,1)*f12(1,1)] + I(1,1)*dW(1,1);


disp('Torque in link 1:');
vpa(subs(Torque(1,1),0.1))
disp('Torque in link 2:');
vpa(subs(Torque(1,2),0.1))
disp('Torque in link 3:');
vpa(subs(Torque(1,3),0.1))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    