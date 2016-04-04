%Old Locokit motors of 24V
%Wnom=9310/19;
%tauStall=0.0228*19;
%New, 40W motors
%Wnom=41400/128;
%tauStall= 1.2;
%Current Locokit motors overriden at 24V
Wnom=7970*2/19;
tauStall=0.0157*2*19;

figure
hold on
for Wmotor=0:round(Wnom)/120:round(Wnom)
    tauOut = tauStall-Wmotor*tauStall/Wnom;
    plot(Wmotor,tauOut, 'k*')
end
gearRatio = 2.1;
for i=24:25
   plot(W(i,1)/gearRatio,abs(Tau(i,1)*gearRatio), 'r*'); 
   plot(W(i,2),abs(Tau(i,2)), 'g*'); 
   plot(W(i,3),abs(Tau(i,3)), 'b*'); 
end

hold off