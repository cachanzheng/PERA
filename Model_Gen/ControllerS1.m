function [taumotor2 u1 u2 tau1 tau2 v1 v2 gravity KpTheta KdTheta]=ControllerS1(theta1,theta2,eTheta1,eTheta2,dtheta1,dtheta2,itheta,iphi)

%Controller that calculates the torque needed to reach the objective and than calculates the corresponding reference value. 


%Degrees to Rad:
theta1=theta1/180*pi;
theta2=theta2/180*pi;
dtheta1=dtheta1/180*pi;
dtheta2=dtheta2/180*pi;
% iTheta=itheta/180*pi;
% iPhi=iphi/180*pi;

%System Parameters
Lc=0.25;       %Link length to centre of mass. Total link length = 0.8
m=3.9;          %Mass
Fc1=0.1; % USE NEW VALUES IF COMPENSATION IS WANTED!
Fc2=0.1; % USE NEW VALUES IF COMPENSATION IS WANTED!
e=1e-9;


KpTheta=[7 0;0 7];    %Proportional gain

KdTheta=[6 0;0 5];      %Differential gain

G =[(981*Lc*m*cos(theta2)*sin(theta1))/100;
 (981*Lc*m*cos(theta1)*sin(theta2))/100];

gravity = G;


theta = [theta1;theta2];
dtheta = [dtheta1;dtheta2];

etheta = [eTheta1;eTheta2];

%friction compensation
%Coulomb

F = [Fc1*(dtheta1)/((dtheta1^2+e))^(1/2);
      Fc2*(dtheta2)/((dtheta2^2+e))^(1/2)];

C = [-Lc^2*dtheta1*dtheta2*m*sin(2*theta2);
    (Lc^2*dtheta1^2*m*sin(2*theta2))/2];



%old torque:
%tau = 900*(-KpTheta*(theta-etheta) - KdTheta*dtheta + G)- 4500*sign(theta-etheta).*(abs(theta-etheta) > 5*pi/180) - 4500*(theta-etheta)/(5*pi/180).*(abs(theta-etheta) <= 5*pi/180);

tau = -KpTheta*(theta-etheta) - KdTheta*dtheta + G;



tau1 = 0;%tau(2);
tau2 = tau(1);

    
taumotor2 = tau2;%+tau2;      



kn=178;     %178    Shoulder    407     Elbow   Speed Constant
R=2.52;     %2.52   Shoulder    2.19    Elbow   Terminal Resistance                      
km=53.8*10^-3;    %53.8   Shoulder    23.4    Elbow   Torque Constant                       
i=550;       %66     Shoulder    66      Elbow   Gear Ratio            
eta=0.7;    %0.70   Shoulder    0.75    Elbow   Max Eff of the Gear



%For light objects in the gripper a reference value of 16000 is 
%enough to reach the whole domain of S1 and S2
vmax=16000;

v1=0;

%poly graad 2 benadering tussen I en Xref:
I=abs(taumotor2)/(i*eta*km)*10^(3) - 70*sign(theta1-eTheta1);
v2=(-0.0613*I^2+64.706*I-229.17)*sign(taumotor2);


%lineaire benadering voor verband I en Xref:
% v2 = ((abs(taumotor2)/(km*10^-3*i*eta)-70)*35.3+4000)*-sign(theta1-eTheta1);

% if abs(v2)<4500 && abs(theta1-eTheta1) > 2*pi/180
%     v2 = -4500*sign(theta1-eTheta1) - v2;
% elseif abs(theta1-eTheta1) < 2*pi/180
%     Kdd = KdTheta*dtheta;
%     taumotor2 = G(1) - Kdd(1);
%     I=abs(taumotor2)/(i*eta*km)*10^(3);
%     v2=(-0.0613*I^2+64.706*I-229.17);
% end




u1=-v2;
u2=v2;   



Final1=16000;
Final2=16000;

if u1>Final1,
    u1=Final1;
elseif u1<-Final1,
    u1=-Final1;
end

if u2>Final2,
    u2=Final2;
elseif u2<-Final2,
    u2=-Final2;
end