%TOES DISPLACEMENT AS A FUNCTION OF TIME 
syms t;
toolY = toolYo + ((toolYf - toolYo)/(tf-to))*(t - to);
toolTheta = toolThetaO + ((toolThetaF - toolThetaO)/(tf-to))*(t - to);
%Distance from the joint to the motor position (percentage of link length)
motorPos=2/4;


%Leg stretched after push-off on running
%toolY = 0.56;
%toolTheta = -pi/8;
%Leg bent before push-off on running
%toolY = 0.4275;
%toolTheta = -pi/2;

%DIRECT AND INVERSE KINEMATICS MODELS
%ANALYSIS OF POSITIONS, VELOCITIES AND ACCELERATIONS IN CARTESIAN AND JOINT
%SPACE FOR ALL THE JOINTS
Kinematics;

%NEWTON-EULER ALGORITHM FOR DYNAMIC MODEL CONSTRUCTION
NewtonEulerAnalysis

figure
xlabel('Time (s)');
ylabel('Absolute value of torque (Nm)');
hold on
h=0;

for ti=0:0.02:tf
    if(h==0)
        h=h+1;
        Tau(h,1)=abs(vpa(subs(Torque(1,1),ti), 3));
        Tau(h,2)=abs(vpa(subs(Torque(1,2),ti), 3));
        Tau(h,3)=abs(vpa(subs(Torque(1,3),ti), 3));
    else
        h=h+1;
        Tau(h,1)=abs(vpa(subs(Torque(1,1),ti), 3));
        Tau(h,2)=abs(vpa(subs(Torque(1,2),ti), 3));
        Tau(h,3)=abs(vpa(subs(Torque(1,3),ti), 3));
        x1=[Tau(h-1,1), Tau(h,1)];
        y1=[ti-0.02, ti];
        x2=[Tau(h-1,2), Tau(h,2)];
        y2=[ti-0.02, ti];
        x3=[Tau(h-1,3), Tau(h,3)];
        y3=[ti-0.02, ti];
        hip=plot(y1,x1,'r');
        k=plot(y2,x2,'g');
        a=plot(y3,x3,'b');
    end
end
legend([hip,k,a],'Hip','Knee','Ankle');
hold off 

        
        
        
% %SOLVE SYMBOLIC EQUATIONS: 
% %TORQUE(t)
% % disp('--------------------------------------');
% % disp('Torque in link 1:');
% Tau(h,1)=vpa(subs(Torque(1,1),ti), 3);
% plot(ti,Tau(h,1),'r*');
% %disp('Torque in link 2:');
% Tau(h,2)=vpa(subs(Torque(1,2),ti), 3);
% plot(ti,Tau(h,2),'g*');
% %disp('Torque in link 3:');
% Tau(h,3)=vpa(subs(Torque(1,3),ti), 3);
% plot(ti,Tau(h,3),'b*');
% %disp('--------------------------------------');


%ANGULAR DISPLACEMENTS AND VELOCITIES OF THE JOINTS
% disp('Initial angular position of joint 1 (degrees):');
% vpa((subs(Q1, 0))*360/(2*pi), 6)
% disp('Initial angular position of joint 2 (degrees):');
% vpa((subs(Q2, 0))*360/(2*pi), 6)
% disp('Initial angular position of joint 3 (degrees):');
% vpa((subs(Q3, 0))*360/(2*pi), 6)
h=0;
figure
xlabel('Time (s)');
ylabel('Joint velocity (rpm)');
hold on

for ti=0:0.02:tf
    if(h==0)
        h=h+1;
        W(h,1)=vpa((abs(subs(Q1, 0))/ti)*60/(2*pi), 6);
        W(h,2)=vpa((abs(subs(Q2, 0))/ti)*60/(2*pi), 6);
        W(h,3)=vpa((abs(subs(Q3, 0))/ti)*60/(2*pi), 6);
    else
        h=h+1;
        W(h,1)=vpa((abs(subs(Q1, ti)-subs(Q1, ti-0.02))/0.02)*60/(2*pi), 6);
        W(h,2)=vpa((abs(subs(Q2, ti)-subs(Q2, ti-0.02))/0.02)*60/(2*pi), 6);
        W(h,3)=vpa((abs(subs(Q3, ti)-subs(Q3, ti-0.02))/0.02)*60/(2*pi), 6);
        x1=[W(h-1,1), W(h,1)];
        y1=[ti-0.02, ti];
        x2=[W(h-1,2), W(h,2)];
        y2=[ti-0.02, ti];
        x3=[W(h-1,3), W(h,3)];
        y3=[ti-0.02, ti];
        hip=plot(y1,x1,'r');
        k=plot(y2,x2,'g');
        a=plot(y3,x3,'b');
    end


% %disp('--------------------------------------');
% %disp('Angular speed of hip motor (rpm):');
% W(h,1)=vpa((abs(subs(Q1, 0))/ti)*60/(2*pi), 6);
% % %disp('Angular speed of knee motor (rpm):');
% W(h,2)=vpa((abs(subs(Q2, 0))/ti)*60/(2*pi), 6);
% % %disp('Angular speed of ankle motor (rpm):');
% W(h,3)=vpa((abs(subs(Q3, 0))/ti)*60/(2*pi), 6);
end
legend([hip,k,a],'Hip','Knee','Ankle');
hold off 

