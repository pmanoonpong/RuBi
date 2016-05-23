%DYNAMICS OF VERTICAL JUMP: impulse analysis
function [Fmin, tmax, F, T]=impulse(hDelta, m, dispCoG)

figure
hold on
for hDelta = 0.01:0.02:0.1
%Total mass of legs
mTotal = 1;
gravity = -9.81;
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
T = linspace(0.01,round(tmax, 2),round(tmax/0.01, 2));
i=0;

for t=0.01:0.01:round(tmax, 2)
    if(t==0.01)
        tprev=t;
        i=i+1;
        F(1,i) = (mTotal* vDelta)/t;
    else
        tnow=t;
        i=i+1;
        F(1,i) = (mTotal* vDelta)/tnow;
        x=[F(1,i-1),F(1,i)];
        y=[tprev, tnow];
        plot(y,x, 'k');
        tprev = t;
    end 
end
%title('Force-Time Plot')
xlabel('Application time (seconds)')
ylabel('Force applied (Newtons)')
% x=[-round(tmax, 2), round(tmax, 2)];
% y=[0, 0];
% plot(x,y,'k');
% y=[150, -150];
% x=[0, 0];
% plot(x,y,'k');
end

hold off

% axis([-0.3 0.3 -0.6 0])
% x1=[vpa(subs(P1(1,1), 0)),vpa(subs(P2(1,1),0))];
% y1=-[vpa(subs(P1(2,1), 0)),vpa(subs(P2(2,1),0))];
% plot(x1,y1)
% x2=[vpa(subs(P2(1,1), 0)),vpa(subs(P3(1,1),0))];
% y2=-[vpa(subs(P2(2,1), 0)),vpa(subs(P3(2,1),0))];
% plot(x2,y2)
% x3=[vpa(subs(P3(1,1), 0)),vpa(subs(P4(1,1),0))];
% y3=-[vpa(subs(P3(2,1), 0)),vpa(subs(P4(2,1),0))];
% plot(x3,y3)