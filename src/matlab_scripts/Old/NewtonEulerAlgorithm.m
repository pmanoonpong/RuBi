function[TORQUE] = NewtonEulerAlgorithm ()
%n, Fext, Next, L, m, dQ, ddQ, Positions
%Inputs: 
%n = number of joints 
%L[Li] = Vector with the lenghts of the links
%m[mi] = Vector with the masses of the links
%Fext = External force applied to the K. chain
%Next = External torque applied to the K. chain
%dQ = Vector with the joints velocities
%ddQ = Vector with the joints accelerations
%Positions = Array of vectors with the position vectors of every joint
%(symbolic) obtained from the directKinematics function

syms L1 L2 L3 m1 m2 m3 dQ1 dQ2 dQ3 ddQ1 ddQ2 ddQ3
syms FextX FextY FextZ NextX NextY NextZ 
syms PinitialX PinitialY PinitialTheta PfinalX PfinalY PfinalTheta
n=3;
m=[m1;m2;m3];
L=[L1;L2;L3];
dQ=[dQ1;dQ2;dQ3];
ddQ=[ddQ1;ddQ2;ddQ3];
Fext=[FextX; FextY; FextZ];
Next=[NextX; NextY; NextZ];
toolPoseInit=[PinitialX; PinitialY; PinitialTheta];
toolPosefinal = [PfinalX PfinalY PfinalTheta];

syms W11 W12 W13 W21 W22 W23 W31 W32 W33; 
W=[W11,W12,W13;W21,W22,W23;W31,W32,W33];
syms dW11 dW12 dW13 dW21 dW22 dW23 dW31 dW32 dW33;
dW=[dW11,dW12,dW13;dW21,dW22,dW23;dW31,dW32,dW33];
syms V11 V12 V13 V21 V22 V23 V31 V32 V33; 
V=[V11,V12,V13;V21,V22,V23;V31,V32,V33];
syms dV11 dV12 dV13 dV21 dV22 dV23 dV31 dV32 dV33;
dV=[dV11,dV12,dV13;dV21,dV22,dV23;dV31,dV32,dV33];

syms Torque1 Torque2 Torque3
TORQUE=[Torque1;Torque2;Torque3];
syms Fpost11 Fpost12 Fpost13 Fpost21 Fpost22 Fpost23 Fpost31 Fpost32 Fpost33
Fpost= [Fpost11 Fpost12 Fpost13; Fpost21 Fpost22 Fpost23; Fpost31 Fpost32 Fpost33];
syms Npost11 Npost12 Npost13 Npost21 Npost22 Npost23 Npost31 Npost32 Npost33
Npost=[Npost11 Npost12 Npost13; Npost21 Npost22 Npost23; Npost31 Npost32 Npost33];

%NullVec=zeros(3,1);

[angularPositions]=invKinematics(toolPoseInit,L);

[Positions]=directKinematics(angularPositions, L);


%Current moments of inertia are assumed to be for solid cilinders of radius r= 0.2 m
r= 2;
%Preallocation of vectors
NullVec=zeros(3,1);

%Forward step
for i = 1:n
    if i==1
        LinkRelPosi_i1 = Positions(:,i) - NullVec;
        [W(:,i), dW(:,i), V(:,i), dV(:,i)] = forwardNEStep(NullVec, NullVec, NullVec, NullVec, dQ(i,1), ddQ(i,1), L(i,1), LinkRelPosi_i1);
    else
        LinkRelPosi_i1 = Positions(:,i)-Positions(:,i-1);
        [W(:,i), dW(:,i), V(:,i), dV(:,i)] = forwardNEStep(W(i-1,:), dW(i-1,:), V(i-1,:), dV(i-1,:), dQ(i,1), ddQ(i,1), L(i,1), LinkRelPosi_i1);
    end
end

%Backwards step
for j = n:-1:1
    %Inertia moment of the link
    I=[(1/12)*m(j,1)*(3*r^2+L(j,1)^2),0,0;0,(1/2)*m(j,1)*r^2,0;0,0,(1/12)*m(j,1)*(3*r^2+L(j,1)^2)];
    LinkRelPosi_i1 = (Positions(:,j+1)-Positions(:,j))/2;
    if i==n
        [TORQUE(:,j), Fpost(:,j), Npost(:,j)] = backwardsNEStep(Fext, Next, I, L(j,1), m(j,1), W(j,1), dW(j,1), dV(j,1), LinkRelPosi_i1);
    else
        [TORQUE(j,1), Fpost(:,j), Npost(:,j)] = backwardsNEStep(Fpost(j-1,:), Npost(j-1,:), I, L(j,1), m(j,1), W(j,:), dW(j,:), dV(j,:), LinkRelPosi_i1);
    end
end
