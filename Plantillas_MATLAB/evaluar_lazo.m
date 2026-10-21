function r = evaluar_lazo(Gc, Gp, Gd, H, tfin, td, titulo)
%EVALUAR_LAZO  Respuesta del lazo simple ante escalón en la referencia (t = 1)
%   y escalón en la perturbación (t = td), con los 3 gráficos que suelen pedir:
%   (1) referencia y salida, (2) acción de control, (3) error.
%     Y = Gc Gp/(1+Gc Gp H) R + Gd/(1+Gc Gp H) D
%   Gd : FT desde la perturbación hasta la salida en lazo abierto
%        (si D entra a la entrada de la planta, Gd = Gp).
%   Gc debe ser PROPIO para calcular u: use pid(P,I,D,Tf) con Tf = 1/N.
if nargin < 4 || isempty(H), H = 1; end
if nargin < 7, titulo = ''; end
t = linspace(0, tfin, 5000)';
ref = double(t >= 1);  dis = double(t >= td);
Tr = feedback(Gc*Gp, H);              % Y/R
Td = Gd*feedback(1, Gc*Gp*H);         % Y/D
Ur = feedback(Gc, Gp*H);              % U/R
Ud = -Gc*H*Td;                        % U/D
y = lsim(Tr, ref, t) + lsim(Td, dis, t);
u = lsim(Ur, ref, t) + lsim(Ud, dis, t);
e = ref - y;
figure('Name', titulo);
subplot(3,1,1), plot(t, ref, 'k--', t, y), grid on, legend('ref','salida'), title([titulo ' - referencia y salida'])
subplot(3,1,2), plot(t, u), grid on, title('Acción de control')
subplot(3,1,3), plot(t, e), grid on, title('Error'), xlabel('t')
S = stepinfo(Tr);
r.Mp = S.Overshoot;  r.ts = S.SettlingTime;
r.ess_ref  = 1 - dcgain(Tr);                  % escalón en la referencia
r.yss_pert = dcgain(Td);                      % desviación final por perturbación unitaria
r.umax = max(abs(u));
fprintf('%s: Mp = %.1f%%  ts = %.3g  e_ss(ref) = %.3g  y_ss(pert) = %.3g  |u|max = %.3g\n', ...
        titulo, r.Mp, r.ts, r.ess_ref, r.yss_pert, r.umax);
end
