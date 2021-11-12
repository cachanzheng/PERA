function [u1 u2 tau1 tau2 v1 v2 gravity KpTheta KdTheta vv1 vv2]=ControllerS12(theta1,theta2,eTheta1,eTheta2,dtheta1,dtheta2,itheta,iphi)

%Controller that calculates the torque needed to reach the objective. 


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


KpTheta=[15 0;0 15];    %Proportional gain

KdTheta=[10 0;0 10];      %Differential gain

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

tau1 = tau(2);
tau2 = tau(1);

taumotor1 = tau1-tau2;      
taumotor2 = tau1+tau2;      



kn=178;     %178    Shoulder    407     Elbow   Speed Constant
R=2.52;     %2.52   Shoulder    2.19    Elbow   Terminal Resistance                      
km=53.8*10^-3;    %53.8   Shoulder    23.4    Elbow   Torque Constant                       
i=550;       %66     Shoulder    66      Elbow   Gear Ratio            
eta=0.7;    %0.70   Shoulder    0.75    Elbow   Max Eff of the Gear



%For light objects in the gripper a reference value of 16000 is 
%enough to reach the whole domain of S1 and S2
vmax=16000;

v1 = (taumotor1/(km*10^-3*i*eta)-60)/0.0308+4000;
v2 = (taumotor2/(km*10^-3*i*eta)-60)/0.0308+4000;

vv1=v1;
vv2=v2;

if abs(v1)>abs(v2);
   ratio1=v1/v2;
   if v1>vmax;
    v1=vmax;
   elseif v1<-vmax;
    v1=-vmax;
   end
   v2=v1/ratio1;
elseif abs(v1)<abs(v2);
   ratio2=v2/v1;
   if v2>vmax;
    v2=vmax;
   elseif v2<-vmax;
    v2=-vmax;
   end
   v1=v2/ratio2;
elseif v1==v2;
    if v1>vmax;
    v1=vmax;
    v2=vmax;
    elseif v1<-vmax;
    v1=-vmax;
    v2=-vmax;
    end
end

    
u1=v1;
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