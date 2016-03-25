%Stand-upstraight phase (no movement)
%Inputs:
%d = 
deltaQ3 = 0.907;
%torqueBody = Fgx*d;

K2 = Fgx*dist/deltaQ3;
%Midstance phase: Energy storage
deltaLeq = 0.13;
mTotal= 0.971;
Froud = 0.2;
Vcom = sqrt(Froud*(L(1,1)+L(2,1))*g)
deltaQ2 = pi/4;
CteEnergy = 0.5;
Keq = ((1/2)*mTotal*Vcom^2)*2*(CteEnergy)/deltaLeq^2
%K3 = (Keq*deltaLeq - K2*deltaQ2)/deltaQ3