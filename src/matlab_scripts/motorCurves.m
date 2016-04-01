%Ratio=198

Wnom=171;
tauStall=0.9;

figure
hold on
for Wmotor=0:10:170
    tauOut = tauStall-Wmotor*tauStall/Wnom;
    plot(Wmotor,tauOut/2, 'k*')
end

for i=30:30
   plot(W(i,1),abs(Tau(i,1)), 'r*'); 
   plot(W(i,2),abs(Tau(i,2)), 'g*'); 
   plot(W(i,3),abs(Tau(i,3)), 'b*'); 
end

hold off