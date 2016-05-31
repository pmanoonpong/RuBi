%TOES TRAJECTORY AS A FUNCTION OF TIME 
syms t;
toolY = toolYo + ((toolYf - toolYo)/(tf-to))*(t - to);
toolTheta = toolThetaO + ((toolThetaF - toolThetaO)/(tf-to))*(t - to);
mP=1/4;

%DIRECT AND INVERSE KINEMATICS MODELS
%ANALYSIS OF POSITIONS, VELOCITIES AND ACCELERATIONS IN CARTESIAN AND JOINT
%SPACE FOR ALL THE JOINTS
Kinematics;

%NEWTON-EULER ALGORITHM FOR DYNAMIC MODEL CONSTRUCTION
NewtonEulerSolution


%TORQUE VALUES FOR THE THREE JOINTS ON THE LEG
figure
xlabel('Time (s)');
ylabel('Absolute value of torque (Nm)');
hold on
h=0;
%Solve the symbolic equations for the given time steps
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

%ANGULAR DISPLACEMENTS AND VELOCITIES OF THE JOINTS (RPM)
h=0;
figure
xlabel('Time (s)');
ylabel('Joint velocity (rpm)');
hold on
%Solve the symbolic equations for the given time steps
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
end
legend([hip,k,a],'Hip','Knee','Ankle');
hold off 

