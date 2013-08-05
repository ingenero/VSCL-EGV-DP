function [egv_t_cum,pre_t_cum] = post_getpos(opt,pre,ns)
% [egv_t_cum,pre_t_cum] = POST_GETPOS(opt,pre,ns)
%
% This function returns vectors with the time the EGV and preceding
% vehicles reach each node (with respect to the starting point).

egv_t_cum = zeros(ns.N+1,1);
for i=1:ns.N+1
    egv_t_cum(i) = sum(opt.t(1:i));
end

pre_length = ns.N+1-pre.firstnode;
pre_t_cum = zeros(pre_length,1);
n = 0;
for i=pre.firstnode:ns.N+1
    n = n+1;
    pre_t_cum(n) = sum(pre.t(pre.firstnode:i));
end