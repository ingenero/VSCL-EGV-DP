function [] = forwardsim(torques)
% [] = FORWARDSIM(torques)
%
% This function calculates the EGV response to a given torque distribution
%
%
% -------------------------------------------------------------------------
% ------------------------------- INPUTS ----------------------------------
%
% -------------------------------------------------------------------------
%
%
% -------------------------------------------------------------------------
% ------------------------------ OUTPUTS ----------------------------------
%
% -------------------------------------------------------------------------

%DEFINE NECESSARY PARAMETERS
avg_vr = (curr_vr+next_vr)/2; %Average speed between current
                    %speed and next speed. This average is
                    %used to calculate the power consumption.
delta_t = delta_s/avg_vr; %The time elapsed when travelling
                    %distance delta_s when accelerating from
                    %curr_vr to next_vr (contantly).
acc = (next_vr-curr_vr)/delta_t; %Acceleration from the crrent
                    %speed to the next speed.

w1 = avg_vr/R_eff; %front rotational speed range
w2 = avg_vr/R_eff; %rear rotational speed range

%based on type, apply correct normal force and power equations
switch terrain.type
    case 'downhill'
        Fn1 = m*g/L*(1/2*cosd(phi)+b*sind(phi)) - m*b/L*acc;
        Fn2 = m*g/L*(1/2*cosd(phi)-b*sind(phi)) + m*b/L*acc;
        P_req = (Ca*avg_vr^2 - m*g*sind(phi) + m*acc)*avg_vr/2;
    case 'level'
        Fn1 = m*g/2 - m*b/L*acc;
        Fn2 = m*g/2 + m*b/L*acc;
        P_req = (Ca*avg_vr^2 + m*acc)*avg_vr/2;
    case 'uphill'
        Fn1 = m*g/L*(1/2*cosd(theta)-b*sind(theta)) - m*b/L*acc;
        Fn2 = m*g/L*(1/2*cosd(theta)+b*sind(theta)) + m*b/L*acc;
        P_req = (Ca*avg_vr^2 + m*g*sind(theta) + m*acc)*avg_vr/2;
    otherwise
        error('Something went wrong with the road slope.')
end
%calculate front/rear power and rear torque
P1 = w1.*T1;
P2 = P_req-P1;
T2 = P2./w2;

%get motor efficiencies at specified speed and torque values
for ii=1:a1
    motorEff01(ii) = getMotorEff(avg_vr,T1(ii),R_eff);
    motorEff02(ii) = getMotorEff(avg_vr,T2(ii),R_eff);
end
%evaluate efficiency based on the torque values
eta01 = zeros(a1,1);
eta02 = zeros(a1,1);
for jj=1:a1
    if T1(jj)<0 && T2(jj)<0;
        powertype = 'regenBoth';
        eta01(jj) = ratioRf*motorEff01(jj); %regenerative brake
        eta02(jj) = ratioRr*motorEff02(jj); %regenerative brake
    elseif T1(jj)<0 && T2(jj)>=0
        powertype = 'regenFront';
        eta01(jj) = ratioRf*motorEff01(jj); %regenerative brake
        eta02(jj) = 1/(ratioDr*motorEff02(jj)); %drive
    elseif T1(jj)>=0 && T2(jj)<0
        powertype = 'regenRear';
        eta01(jj) = 1/(ratioDf*motorEff01(jj)); %drive
        eta02(jj) = ratioRr*motorEff02(jj); %regenerative brake
    elseif T1(jj)>=0 && T2(jj)>=0
        powertype = 'regenNone';
        eta01(jj) = 1/(ratioDf*motorEff01(jj)); %drive
        eta02(jj) = 1/(ratioDr*motorEff02(jj)); %drive
    else
        disp('Something went wrong with the power calculation!')
        break
    end
end
%calculate power
Pg1 = w1.*T1;
Pg2 = w2.*T2;
P_tot = Pg1.*eta01 + Pg2.*eta02;

%POPULATE VECTORS WITH POSSIBLE STATE VALUES (SOE)
%calculate change in state of charge of the battery
delta_SOC = P_tot*delta_t/(E_max*V_bat*h2s);







