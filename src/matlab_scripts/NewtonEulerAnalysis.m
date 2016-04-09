%function[TORQUE] = NewtonEulerAlgorithm2 (n, Fext, Next, L, m)
%Inputs:
%m[mi] = Vector with the masses of the links
%Next = External torque applied to the K. chain
%hDelta = Maximum displacement in Y axis of the body on the jump

%N-E FORWARD STEP

%fij = internal force applied in joint i by link j
%ricj = Distance vector from joint i to CoG of link j

%Link i=3
f23 = fext - m(3,1)*g + m(3,1)*Accelerations(:,4);

r2c3 = (Positions(:,4)-Positions(:,3))*motorPos;
r3c3 = (Positions(:,3)-Positions(:,4))*(1-motorPos);

Torque(1,3) = TauExt + [r2c3(1,1)*f23(2,1)-r2c3(2,1)*f23(1,1)] - [r3c3(1,1)*fext(2,1)-r3c3(2,1)*fext(1,1)] + I(3,1)*dW(3,1);

%Link i=2
f12 = f23 - m(2,1)*g + m(2,1)*Accelerations(:,3);

r1c2 = (Positions(:,3)-Positions(:,2))*motorPos;
r2c2 = (Positions(:,2)-Positions(:,3))*(1-motorPos);

Torque(1,2) = Torque(1,3) + [r1c2(1,1)*f12(2,1)-r1c2(2,1)*f12(1,1)] - [r2c2(1,1)*f23(2,1)-r2c2(2,1)*f23(1,1)] + I(2,1)*dW(2,1);

%Link i=1
f01 = f12 -m(1,1)*g + m(1,1)*Accelerations(:,2);

r0c1 = (Positions(:,2)-Positions(:,1))*(motorPos);
r1c1 = (Positions(:,2)-Positions(:,1))*(1-motorPos);

Torque(1,1) = Torque(1,2) + [r0c1(1,1)*f01(2,1)-r0c1(2,1)*f01(1,1)] - [r1c1(1,1)*f12(2,1)-r1c1(2,1)*f12(1,1)] + I(1,1)*dW(1,1);




    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    