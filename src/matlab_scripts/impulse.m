%DYNAMICS OF VERTICAL JUMP: impulse analysis
function [Fmin, tmax]=impulse(hDelta, m, dispCoG, gravity)

%Total mass of legs
mTotal = 2*[m(1,1)+m(2,1)+m(3,1)];
%Maximum vertical displacement of the body
%hDelta=0.01;
%Body velocity increment
vDelta = sqrt(2*abs(gravity)*hDelta);

%Fmin and tmax for the required Energy
Fmin = (mTotal*vDelta^2)/(2*dispCoG);
tmax = mTotal*vDelta/Fmin;
Fmin=11.21;
tmax=0.15;

%Reciprocal relationship between force and application time for the desired
%hDelta
figure
hold on
for t=0.01:0.002:0.3
    F = (mTotal* vDelta)/t;
    plot(t,F, '*')
end
hold off
