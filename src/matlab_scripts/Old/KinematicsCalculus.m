%Make sure everthing is in grades or radians!!
%*********************************************

%Symbolic inputs to test functionality
syms L1 L2 L3 m1 m2 m3 dQ1 dQ2 dQ3 ddQ1 ddQ2 ddQ3
syms FextX FextY FextZ NextX NextY NextZ 
syms PinitialX PinitialY PinitialTheta PfinalX PfinalY PfinalTheta

m=[m1;m2;m3];
L=[L1;L2;L3];
dQ=[dQ1;dQ2;dQ3];
ddQ=[ddQ1;ddQ2;ddQ3];
Fext=[FextX; FextY; FextZ];
Next=[NextX; NextY; NextZ];
toolPoseInit=[PinitialX; PinitialY; PinitialTheta];
toolPosefinal = [PfinalX PfinalY PfinalTheta];
NullVec=zeros(3,1);


%Initial osition of toes [Px, Py]
%Y3o is an input parameter for the design
toesInitialPos=[0,Y3o];
Y3f = L1+L2+L3; %Leg fully streched
toesFinalPos=[0,Y3f];

%Displacement of the toes
dispP = toesFinalPos - toesInitialPos;
%Desired linear velocity of the toes in dP, dP'= [Vx, Vy]

%Hessian:
%f=[Px; Py];
%H = hessian(f, [Q1, Q2, Q3]);























