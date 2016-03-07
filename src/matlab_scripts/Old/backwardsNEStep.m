%NEWTON-EULER METHOD STEPS 2/2

%%BACKWARD STEP%%
function[TORQUEi, Fi, Ni] = backwardsNEStep(Fpost, Npost, Ii, Li, mi, Wi, dWi, dVi, LinkRelPosi_i1)
%Input parameters: 3x1 vectors Fpost, Npost, 3x3 Matrix I and scalar Li
Zbase_i=[0;0;1];

%Joint forces and torques
Fi = mi*dVi + Fpost; 
Ni = Ii*dWi + cross(Wi,(Ii*Wi)) + Npost + cross((LinkRelPosi_i1),Fi) + cross((LinkRelPosi_i1*2 - (LinkRelPosi_i1)),Fpost);

%For revolute joints, output torque
TORQUEi = Ni*Zbase_i;