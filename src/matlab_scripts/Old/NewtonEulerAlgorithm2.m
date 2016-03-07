%function[TORQUE] = NewtonEulerAlgorithm2 (n, Fext, Next, L, m, dQ, ddQ, Positions)
%Inputs: 
%n = number of joints 
%L[Li] = Vector with the lenghts of the links
%m[mi] = Vector with the masses of the links
%Fext = External force applied to the K. chain
%Next = External torque applied to the K. chain
%toolPoseInit = Initial position of the toes [X,Y, theta]
syms Q1 Q2 Q3 L1 L2 L3 m1 m2 m3 dQ1 dQ2 dQ3 ddQ1 ddQ2 ddQ3
syms FextX FextY FextZ NextX NextY NextZ 
syms PinitialX PinitialY PinitialTheta PfinalX PfinalY PfinalTheta

%SYMBOLIC INPUTS FOR TESTING
n=3;                            %Number of joints in the chain
%syms g;
%m=[m1;m2;m3];
%Fext=[FextX; FextY; FextZ];
%Next=[NextX;NextY;NextZ];
%L=[L1;L2;L3];
%dQ=[dQ1;dQ2;dQ3];
%ddQ=[ddQ1;ddQ3;ddQ3];
%Q=[Q1;Q2;Q3];
%syms r;

%DYNAMICS PARAMETERS
%Inertia moments are computed for cilinders of length L and radius r
r=1;
g=9.81;
m=[1;1;1];
Fext=[0; -(m(1,1)+m(2,1)+m(3,1))*g; 0];
Next=[0;0;0];

%INPUTS FROM KINEMATICS ANALYSIS
Q=[Qo(1,1);Qo(2,1);Qo(3,1)];
dQ=[velQ(1,1);velQ(2,1);velQ(3,1)];
ddQ=[accQ(1,1);accQ(2,1);accQ(3,1)];


%Symbolic variables, only for testing output equations
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

%Direct kinematics
[Positions]=directKinematics(Q, L);

%For our specific case
Zbase_i=[0;0;1];
KSIi= 1;  

%Null vector for first forward step
NullVec=zeros(3,1);

%Forward step
for i = 1:n
    if i==1
        LinkRelPosi_i1 = Positions(:,i) - NullVec;
        Wprev=NullVec;
        dWprev=NullVec;
        Vprev=NullVec;
        dVprev=NullVec;
    else 
        LinkRelPosi_i1 = Positions(:,i)-Positions(:,i-1);
        Wprev=W(:,i-1);
        dWprev=dW(:,i-1);
        Vprev=V(:,i-1);
        dVprev=dV(:,i-1);
    end
    
    %Angular velocity and acceleration
    W(:,i) = Wprev + KSIi*dQ(i,1)*Zbase_i;
    dW(:,i) = dWprev + KSIi*(ddQ(i,1)*Zbase_i + cross((dQ(i,1)*Wprev), Zbase_i));

    %Linear velocity and acceleration
    V(:,i) = Vprev + cross(Wprev, LinkRelPosi_i1) + (1- KSIi)*dQ(i,1)*Zbase_i;
    dV(:,i) = dVprev + cross(dWprev, LinkRelPosi_i1) + cross(Wprev,cross(Wprev,LinkRelPosi_i1)) + (1-KSIi)*[ddQ(i,1)*Zbase_i + cross(2*dQ(i,1)*Wprev, Zbase_i)];
  
end

%Backwards step
for j = n:-1:1
    %Inertia moment of the link
    I=[(1/12)*m(j,1)*(3*r^2+L(j,1)^2),0,0;0,(1/2)*m(j,1)*r^2,0;0,0,(1/12)*m(j,1)*(3*r^2+L(j,1)^2)];
    %I=[1 0 0; 0 1 0; 0 0 1];
    LinkRelPosi_i1 = (Positions(:,j+1)-Positions(:,j))/2;
    if j==n
        Fanterior=Fext;
        Nanterior=Next;
    else
        Fanterior=Fpost(:,j+1);
        Nanterior=Npost(:,j+1);
    end
    
    %Joint forces
    Fpost(:,j) = m(j,1)*dV(:,j) + Fanterior; 
    %Joint torques
    Npost(:,j) = I*dW(:,j) + cross(W(:,j),(I*W(:,j))) + Nanterior + cross((LinkRelPosi_i1),Fanterior) + cross((LinkRelPosi_i1*2 - (LinkRelPosi_i1)),Fanterior);
    %For revolute joints, output torque
    TORQUE(j,1) = transpose(Npost(:,j))*Zbase_i;
end

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    