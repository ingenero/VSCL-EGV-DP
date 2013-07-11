function [] = post_plotHARDCODE(filenames,terrain,opt,pre,egv,param,ns)

cmpdata1 = importdata([filenames.folder filenames.sample1]);
cmpdata2 = importdata([filenames.folder filenames.sample2]);
constdata = importdata([filenames.folder filenames.sample3]);
d_egv = cumtrapz(opt.t_cum,opt.v*param.conv.kmh2mps);
d_pre = cumtrapz(pre.t_cum,pre.v(pre.firstnode:ns.N+1)*param.conv.kmh2mps)-egv.x.step;

cmpdata2.t = zeros(ns.N+1,1);
cmpdata2.t_cum = zeros(ns.N+1,1);
for i=1:ns.N
    v1_avg = (cmpdata1.vr_opt(i+1)+cmpdata1.vr_opt(i))/2;
    v2_avg = (cmpdata2.vr_opt(i+1)+cmpdata2.vr_opt(i))/2;
    cmpdata1.t(i+1) = 30/v1_avg/param.conv.kmh2mps;
    cmpdata2.t(i+1) = 30/v2_avg/param.conv.kmh2mps;
end
for i=1:ns.N+1
    cmpdata1.t_cum(i) = sum(cmpdata1.t(1:i));
    cmpdata2.t_cum(i) = sum(cmpdata2.t(1:i));
end
d1_egv = cumtrapz(cmpdata1.t_cum,cmpdata1.vr_opt*param.conv.kmh2mps);
d2_egv = cumtrapz(cmpdata2.t_cum,cmpdata2.vr_opt*param.conv.kmh2mps);

figure(1);
subplot(411); plot(terrain.dist,opt.SOC,'b'); ylabel('SOC'); hold on
plot(terrain.dist,cmpdata1.SOC,'g')
plot(terrain.dist,cmpdata2.SOC,'r')
plot(terrain.dist,constdata.SOC,'--','Color',[1 .5 0])

subplot(412); hold on;
plot(terrain.dist,pre.v,'--','Color',[1 .5 0])
plot(terrain.dist,cmpdata1.vr_opt,'-og')
plot(terrain.dist,cmpdata2.vr_opt,'r')
plot(terrain.dist,opt.v,'b'); ylabel('Velocity (km/h)'); 

subplot(413); hold on
% plot(d1_egv,cmpdata1.t_cum,'-g')
% plot(d2_egv,cmpdata2.t_cum,'r')
plot(d_egv,opt.t_cum,'b')
plot(d_pre,pre.t_cum,'--','Color',[1 .5 0])
% plot(opt.t_cum,terrain.dist,'oc')
% plot(pre.t_cum,terrain.dist(pre.firstnode:ns.N+1),'o','Color',[1 .5 0])
ylabel('Time (s)'); xlim([0 3000])

subplot(414); plot(terrain.dist,terrain.alti); ylabel('Altitude (m)')
xlabel('Distance (m)')