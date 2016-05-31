%JACOBIAN FOR DYNAMIC MODEL
%Currently under development

function []=jacobianUse(InitToolPose,Qf)
%Inputs:
%InitlToolPose= 2x1 Vector containing [X,Y] position of the end-effector (symbolic)
%Qo = initial joint configuration
%Qf = final joint configuration (in our case Qf=[0;0;0];)

%Jacobian from kinematics model: J(Qo)
syms Q1 Q2 Q3
J = jacobian([InitToolPose(1,1); InitToolPose(2,1)], [Q1, Q2, Q3]);
Qo=[Q1; Q2; Q3];
%Pseudo inverted Jacobian (Moore-Penrose method)
Jinv=pinv(J);
%Check that J*Jinv yields the unity matrix
display('If the jacobian is correct, we get the identity matrix')
simplify(J*Jinv)

%For testing the inverse kinematics result dispP=J*dispQ  
dispQ=Qo-Qf;
dispP=J*dispQ;
display(dispP,'Cartesian displacement of the tool')

%Derive the position displacement to obtain the linear speed %dP=J*dQ --> dQ=Jinv*dP
%Angular velocities
%dispPsymb = InitToolPose - FinalToolPose;
%dispPsymb = [dispPsymb(1,1); dispPsymb(2,1)];

%ddispPsymb = diff(dispPsymb,);
%dQ = Jinv* ddispPsymb;
