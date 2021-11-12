function [taumotor2 gravity dEncTheta dEncPhi EncPhi EncTheta dTheta1 dTheta2 EncX EncY Theta Phi dTheta dPhi dThetaX dThetaY ControlSigX ControlSigY Time Ms_1 Ms_2]=ControlS1(eTheta,ePhi,ExecTime)

%Control algorithm to control the joints S1 and S2 of PERA. The inputs are
%eTheta1: expected value of theta_1 (S1) in radians
%eTheta2: expected value of theta_2 (S2) in radians
%ExecTime: execution time

LoopTime = 0.01;     % Sampling Period

 for i=1:(ExecTime-LoopTime)/LoopTime %Initializing variables
     Theta(i)=0;
     Phi(i)=0;
 
     dThetaX(i)=0;
     dThetaY(i)=0;
     
     itheta(i)=0;
     iphi(i)=0;
     
     dTheta1(i)=0;
     dTheta2(i)=0;
     dTheta(i)=0;
     dPhi(i)=0; 
     dEncTheta(i)=0;
     dEncPhi(i)=0;
     TdTheta(i)=0;
     TdPhi(i)=0; 
     EncX(i)=0;
     EncY(i)=0;

     TTdTheta(i)=0;
     TTdPhi(i)=0;
     TTdEncTheta(i)=0;
     TTdEncPhi(i)=0;
     Ms_1(i)=0;
     Ms_2(i)=0;
end;


i = 1;
tic; 
while (i<(ExecTime/LoopTime)),
    while toc<LoopTime, end                         % busy wait until period is reached
    SampTimeArray(i) = toc;                         % store sampling time
    tic;                                            % start timer again
    [success, Enc, ADC, MsgIndex] = rtm_usb_ReadStdMsg(0);
    Enc = double(Enc);
    ADC = double(ADC);

%   TEST CODE: 
    Ms_10(i)=ADC(1);
    Ms_11(i)=ADC(2);
    Ms_12(i)=ADC(3);
    Ms_13(i)=ADC(4);
    Ms_14(i)=ADC(5);
    Ms_15(i)=ADC(6);
    Ms_16(i)=ADC(7);
%   END TEST CODE

    EncX(i) = Enc(1);
    EncY(i) = Enc(2);

    
%   Correcting for jumps in encoder:

    if EncX(i)>10^7
        EncX(i) = EncX(i)-4.294967275000000*10^9+40;
    end
    
    if EncY(i)<10^7
        EncY(i)=EncY(i)+4.294967186000000*10^9-16;
    end
    EncX(i) = EncX(i) + 1744 - 3399;
    EncY(i) = EncY(i)-4.294966341000000e+009 + 208 - 3127;
    
    EncPhi(i) = 0.5*(EncX(i) + EncY(i));
    EncTheta(i) = 0.5*(EncY(i) - EncX(i));
    
    % Converting encoder to degrees
    EncPhi(i) = EncPhi(i)*6.341209938079804e-004 ;
    EncTheta(i) = (EncTheta(i) + 5.672000000238419e+003)*(6.367308821541894e-004);
    
    if EncTheta(i) < 0
        EncTheta(i) = 0;
    end
    if EncTheta(i) > 90
        EncTheta(i) = 90;
    end
    if EncPhi(i) < -90
        EncPhi(i) = -90;
    end
    if EncPhi(i) > 90
        EncPhi(i) = 90;
    end
    
    %Calculation of the derivative of the encoder angles
    
    if i > 2
        TTdEncTheta(i) = (EncTheta(i)-EncTheta(i-1))/SampTimeArray(i);        
        if i > 10
            dEncTheta(i) = mean(TTdEncTheta(i-10:i));
        else
            dEncTheta(i) = 0;
        end
        TTdEncPhi(i) = (EncPhi(i)-EncPhi(i-1))/SampTimeArray(i);
        if i > 10
            dEncPhi(i) = mean(TTdEncPhi(i-10:i));
        else
            dEncPhi(i) = 0;
        end
    end
    
    
Theta(i) = EncTheta(i);
Phi(i) = EncPhi(i);
dTheta(i) = dEncTheta(i);
dPhi(i) = dEncPhi(i);

theta1 = Theta(i);
theta2 = Phi(i);
dtheta1 = dTheta(i);
dtheta2 = dPhi(i);
eTheta1 = eTheta;
eTheta2 = ePhi;

    %Controller
    [taumotor2 u1 u2 tau1 tau2 v1 v2 gravity KpTheta KdTheta]=ControllerS1(theta1,theta2,eTheta1,eTheta2,dtheta1,dtheta2,itheta,iphi);
    successsSend = rtm_usb_SendStdMsg(0,0,[u1 u2],0);  % send reference signal Channel X right now
    % TEST CODE:        
    %successsSend = rtm_usb_SendStdMsg(0,0,[0 0],0);
    
    ControlSigX(i)=u1;              %To Plot the Control Signal Total
    ControlSigY(i)=u2;
    
    Tau1(i)=tau1;
    Tau2(i)=tau2;
    
    VoltageX(i)=v1;
    VoltageY(i)=v2;
    
    Gravity(i,:)=gravity;          %To Plot the Effects due to Gravity

    %KpTheta(i)=kpTheta;            %To Plot the Effects due to Kp
    %KpPhi(i)=kpPhi;
 
    i = i+1;
end



[REncX CEncX]=size(EncX);
for i=1:CEncX
    EncT(i)=1/2*(EncX(i)+EncY(i));
end;
EncT=double(EncT);


Time = 0:LoopTime:(ExecTime-2*LoopTime); 

end