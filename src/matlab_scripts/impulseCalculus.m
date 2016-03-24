function [F]=motors(t)
%DYNAMICS OF VERTICAL JUMP: impulse analysis
% 
% %Maximum vertical displacement of the body on jump
% syms hDelta
% hDelta = 0.05;
% %Gravity constant
% gravity = -9.81;
% %Total mass of legs
% m=[0.35;0.25;0.15];
% mTotal = 2*[m(1,1)+m(2,1)+m(3,1)];
% %Initial velocity of body
% Vinit = 0;
% Ttakeoff = 0.2;
% 
% %Velocity at takeoff
% Vtakeoff = sqrt(2*abs(gravity)*hDelta);
% vDelta = Vtakeoff - Vinit;
% 
% %Impulse 
% I= mTotal*vDelta;
% 
% 
% 
% 
% 
% 
% 
% Fg = ((2*Vtakeoff/Ttakeoff) - gravity) * mTotal;
% 
% Tfg = mTotal * vDelta/Fg;


gravity = -9.81;
%Total mass of legs
m=[0.35;0.25;0.15];
mTotal = 2*[m(1,1)+m(2,1)+m(3,1)];
hDelta=0.01;
%t = linspace(0,5,0.01);
F = (mTotal* sqrt(2*abs(gravity)*hDelta))/t;
