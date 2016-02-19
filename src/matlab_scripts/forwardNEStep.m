%NEWTON-EULER METHOD STEPS 1/2

%%FORWARD STEP%%
function [Wi, dWi, Vi, dVi] = forwardNEStep(Wprev, dWprev, Vprev, dVprev, dQi, ddQi, Li, LinkRelPos)
%Input parameters: 3x1 vectors Wprev, dWprev, Vprev, dVprev and scalars
%dQi, ddQi and Li LinkRelPos
%Common parameters for all the joints (for our case)
Zbase_i=[0;0;1];
KSIi= 1;  

%Angular velocity and acceleration
Wi = Wprev + KSIi*dQi*Zbase_i;
dWi = dWprev + KSIi*(ddQi*Zbase_i + cross((dQi*Wprev), Zbase_i));

%Linear velocity and acceleration
Vi = Vprev + cross(Wprev, LinkRelPos) + (1- KSIi)*dQi*Zbase_i;
dVi = dVprev + cross(dWprev, LinkRelPos) + cross(Wprev,cross(Wprev,LinkRelPos)) + (1-KSIi)*[ddQi*Zbase_i + cross(2*dQi*Wprev, Zbase_i)];





