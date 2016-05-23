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
xlabel('Rotational speed (rpm)');
ylabel('Torque (Nm)');

hold on

i=0;
for Wmotor=0:round(Wnom)/120:round(Wnom)
    if(Wmotor==0)
        WmotorPrev=Wmotor;
        i=i+1;
        tauOut(1,i) = tauStall-Wmotor*tauStall/Wnom;
    else
        WmotorNow=Wmotor;
        i=i+1;
        tauOut(1,i) = tauStall-WmotorNow*tauStall/Wnom;
        x=[tauOut(1,i-1), tauOut(1,i)];
        y=[WmotorPrev, WmotorNow];
        plot(y,x, 'k');
        WmotorPrev=WmotorNow;
    end
end

gearRatio = 2;
for i=10:size(W)
   h=plot(W(i,1)/gearRatio,abs(Tau(i,1)*gearRatio), 'r*'); 
   k=plot(W(i,2),abs(Tau(i,2)), 'g*'); 
   a=plot(W(i,3),abs(Tau(i,3)), 'b*'); 
end

legend([h,k,a],'Hip','Knee','Ankle');

hold off
