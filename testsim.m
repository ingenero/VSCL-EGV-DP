close all; clc

folder = 'inputdata/';
filename1 = 'DP_data_25-33_ini26-fin26.mat';
filename2 = 'DP_data_pre28_x0-10.mat';
d = importdata([folder filename2]);
s = 0:30:3000;

%set limits on torques
tlim.hi =  30; %[N-m]
tlim.lo = -35;
torques.T1 = zeros(length(s),1);
torques.T2 = zeros(length(s),1);
for i=1:length(s)
    if d.T1_opt(i) > tlim.hi
        torques.T1(i) = tlim.hi;
    elseif d.T1_opt(i) < tlim.lo
        torques.T1(i) = tlim.lo;
    else
        torques.T1(i) = d.T1_opt(i);
    end
    
    if d.T2_opt(i) > tlim.hi
        torques.T2(i) = tlim.hi;
    elseif d.T2_opt(i) < tlim.lo
        torques.T2(i) = tlim.lo;
    else
        torques.T2(i) = d.T2_opt(i);
    end
end

egv.v.v0 = d.vr_opt(1);
[v,SOC] = forwardsim(torques,terrain,egv,param);
comb = horzcat(v.sol,d.vr_opt);

%%
figure(1);
subplot(311); hold on
plot(s,SOC,'-r')
plot(s,d.SOC,'b')
ylabel('SOC')
legend('forward','optimal')

subplot(312); hold on
plot(s,v.sol,'-r')
plot(s,d.vr_opt,'b')
ylabel('Velocity (km/h)')

subplot(313); hold on
plot(s,torques.T1,'-or')
plot(s,torques.T2,'-o','Color',[1 0.5 0.2])
plot(s,d.T1_opt,'b')
plot(s,d.T2_opt,'c')
ylabel('Torque (N-m)')
xlabel('Distance (m)')