function m = metricas26(t, y, u, ref, t_d)
% METRICAS26  Sobrepaso, tiempos de subida/establecimiento y rechazo a la perturbación.
if nargin < 4, ref = 1; end
if nargin < 5, t_d = 80; end
dt = t(2) - t(1);
a = t < t_d; ya = y(a)/ref; ta = t(a);
m.Mp = max(0, (max(ya) - 1)*100);
m.tr = ta(find(ya >= 0.9, 1)) - ta(find(ya >= 0.1, 1));
i = find(abs(ya - 1) > 0.02, 1, 'last'); m.ts = ta(i);
p = t >= t_d; dev = y(p) - ref; tp = t(p);
m.pico = max(dev);
i = find(abs(dev) > 0.02, 1, 'last'); if isempty(i), m.ts_pert = 0; else, m.ts_pert = tp(i) - t_d; end
m.IAE_pert = sum(abs(dev))*dt;
m.e_ss = -dev(end);
m.umax = max(abs(u));
end
