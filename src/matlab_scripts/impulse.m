%DYNAMICS OF VERTICAL JUMP: impulse analysis
function [Fmin, tmax, F, T]=impulse(hDelta, m, dispCoG, gravity)

%Total mass of legs
mTotal = 0.971;
%2*[m(1,1)+m(2,1)+m(3,1)];
%Maximum vertical displacement of the body
%hDelta=0.01;
%Body velocity increment
vDelta = sqrt(2*abs(gravity)*hDelta);

%Fmin and tmax for the required Energy
Fmin = (mTotal*vDelta^2)/(2*dispCoG);
tmax = mTotal*vDelta/Fmin;

%Reciprocal relationship between force and application time for the desired
%hDelta
T = linspace(0.01,0.3,30);
i=0;
figure
hold on
for t=0.01:0.01:0.3
    i=i+1;
    F(1,i) = (mTotal* vDelta)/t;
    plot(t,F(1,i), '*')
end
hold off
