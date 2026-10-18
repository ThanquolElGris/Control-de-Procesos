%% CP-5: Control feedforward y combinación FF-FB
% Simulación con connect + lsim (equivalente al modelo Simulink: todos los bloques son lineales).
clear; clc; close all;
s = tf('s');
set(groot, 'defaultFigureVisible', 'off');
warning('off', 'Control:analysis:LsimUndersampled');
simu = @(M, u, t) lsim(c2d(M, t(2)-t(1), 'zoh'), u, t);   % ZOH exacto para entradas escalón; maneja bien los retardos

%% ======================== EJERCICIO 1 ========================
Gv = 2/(s+2);            % = 1/(0.5s+1)
Gp = 6/(4*s+2);          % = 3/(2s+1)
Kd = 4;                  % la perturbación entra entre Gv y Gp

% ---- a) Módulo óptimo sobre Gv*Gp (Tu = 0.5, se compensa T = 2) ----
G  = Gv*Gp;
Tu = 0.5;
Gc = minreal(1/(2*Tu*s*(Tu*s+1)*G))                 % = (2s+1)/(3s) = 0.6667 + 0.3333/s

% ---- c) Feedforward: Gff = -Gd/(Gm*Gv*Gp), con Gd = Kd*Gp y Gm = 1 ----
Gff_ideal = minreal(-Kd*Gp/(Gv*Gp))                  % = -4(0.5s+1) = -2s - 4 (impropia)
Gff_din   = -Kd*(0.5*s+1)/(0.01*s+1);                % ideal + polo rápido para poder implementarla
Gff_est   = -Kd;                                     % solo ganancia estática

% ---- b) y d) Experimento: r = 2 en t = 1, d = 0.1 en t = 10, 20 unidades ----
t = (0:0.001:20)';
r = 2*(t >= 1);  d = 0.1*(t >= 10);
casos = {0, Gff_est, Gff_din};  nombres = {'solo FB (MO)', 'FB + FF estático', 'FB + FF dinámico'};
f = figure('Position',[100 100 800 650]);
for k = 1:3
    M = lazo1(Gv, Gp, Kd, Gc, casos{k});
    yu = simu(M, [r d], t);
    subplot(2,1,1); hold on; plot(t, yu(:,1));
    subplot(2,1,2); hold on; plot(t, yu(:,2));
    i = t >= 10;  dev = yu(i,1) - 2;
    fprintf('Ej1 %-18s pico ante d = %+.4f  y(20) = %.4f\n', nombres{k}, dev(find(abs(dev)==max(abs(dev)),1)), yu(end,1));
end
subplot(2,1,1); plot(t, r, 'k--'); grid on; ylabel('salida');
legend([nombres, {'referencia'}], 'Location','southeast'); title('CP5 Ej. 1: r = 2 en t = 1, d = 0.1 en t = 10');
subplot(2,1,2); grid on; ylabel('mando u'); xlabel('t');
guardar(f, 'ej1.png');
% Zoom en la perturbación
f = figure('Position',[100 100 800 350]); hold on;
for k = 1:3
    yu = simu(lazo1(Gv, Gp, Kd, Gc, casos{k}), [r d], t);  plot(t, yu(:,1));
end
xlim([9.5 20]); ylim([1.9 2.25]); grid on; legend(nombres); title('Ej. 1: zoom en la perturbación'); xlabel('t');
guardar(f, 'ej1_zoom.png');

%% ======================== EJERCICIO 2 ========================
Gv = 2/(s+4);            % = 0.5/(0.25s+1)
G1 = 4/(4*s+2);          % = 2/(2s+1)       -> salida2
G2 = 6/(4*s+1);  Lp = 1; % G2*e^{-s}        -> salida1
Gd = 2/(s+10);   Ld = 2; % perturbación: Gd*e^{-2s}, entra antes de G1

% ---- a) PID por ZN (lazo cerrado) para salida1 ----
Ga = Gv*G1*G2;  Ga.OutputDelay = Lp;
[Ku,~,wu] = margin(Ga);  Tu = 2*pi/wu;
[Kc,Ti,Td] = zn_pid(Ku, Tu);
fprintf('\nEj2 a) Ku = %.4f  wu = %.4f  Tu = %.4f  ->  ideal: Kc = %.4f  Ti = %.4f  Td = %.4f\n', Ku, wu, Tu, Kc, Ti, Td);
Gc_a = pidstd(Kc, Ti, Td, 10);                       % PID ideal con filtro derivativo N = 10

% ---- c) Cascada: PI interno por MO sobre Gv*G1 (Tu = 0.25), PID externo por ZN ----
Gi  = Gv*G1;
Gci = minreal(1/(2*0.25*s*(0.25*s+1)*Gi))           % = (2s+1)/(0.5s) = 4 + 2/s
Gb  = minreal(feedback(Gci*Gi,1))*G2;  Gb.OutputDelay = Lp;
[Ku2,~,wu2] = margin(Gb);  Tu2 = 2*pi/wu2;
[Kc2,Ti2,Td2] = zn_pid(Ku2, Tu2);
fprintf('Ej2 c) externo: Ku = %.4f  Tu = %.4f  ->  ideal: Kc = %.4f  Ti = %.4f  Td = %.4f\n', Ku2, Tu2, Kc2, Ti2, Td2);
Gce = pidstd(Kc2, Ti2, Td2, 10);

% ---- d) Feedforward: Gff = -Gd*e^{-2s}/(Gm*Gv), Gm = 1 ----
Gff = minreal(-Gd/Gv);  Gff.InputDelay = Ld;         % = -(s+4)/(s+10) e^{-2s}  (realizable)
Gff

% ---- b) y c) Experimento: r = 2 en t = 1, d = 0.1 en t = 30, 60 unidades ----
t = (0:0.01:60)';
r = 2*(t >= 1);  d = 0.1*(t >= 30);
Ms = lazo2(Gv,G1,G2,Lp,Gd,Ld,Gc_a,[],0);  Mc = lazo2(Gv,G1,G2,Lp,Gd,Ld,Gce,Gci,0);
ys = simu(Ms, [r d], t);  yc = simu(Mc, [r d], t);
f = figure('Position',[100 100 800 650]);
subplot(2,1,1); plot(t, ys(:,1), t, yc(:,1), t, r, 'k--'); grid on; ylabel('salida1');
legend('lazo simple (ZN)','cascada (MO + ZN)','referencia','Location','southeast'); title('CP5 Ej. 2: r = 2 en t = 1, d = 0.1 en t = 30');
subplot(2,1,2); plot(t, ys(:,2), t, yc(:,2)); grid on; ylabel('mando u'); xlabel('t'); ylim([-2 12]);
guardar(f, 'ej2_bc.png');
metr('lazo simple', t, ys(:,1), 2, 30);  metr('cascada', t, yc(:,1), 2, 30);

% ---- d) Solo perturbación, con y sin FF ----
r0 = zeros(size(t));
f = figure('Position',[100 100 800 400]); hold on;
etq = {'simple sin FF','simple + FF','cascada sin FF','cascada + FF'};
M = {lazo2(Gv,G1,G2,Lp,Gd,Ld,Gc_a,[],0), lazo2(Gv,G1,G2,Lp,Gd,Ld,Gc_a,[],Gff), ...
     lazo2(Gv,G1,G2,Lp,Gd,Ld,Gce,Gci,0), lazo2(Gv,G1,G2,Lp,Gd,Ld,Gce,Gci,Gff)};
fprintf('\nEj2 d) solo perturbación (d = 0.1 en t = 30):\n');
for k = 1:4
    y = simu(M{k}, [r0 d], t);  plot(t, y(:,1));
    fprintf('  %-16s pico = %+.5f   IAE = %.4f\n', etq{k}, y(find(abs(y(:,1))==max(abs(y(:,1))),1),1), trapz(t, abs(y(:,1))));
end
grid on; xlim([28 60]); legend(etq); xlabel('t'); ylabel('salida1'); title('Ej. 2 d): respuesta a d = 0.1 en t = 30');
guardar(f, 'ej2_d.png');

%% ======================== funciones ========================
function guardar(f, nombre)
    try, theme(f, 'light'); end                     % R2025a+: exportar siempre con fondo claro
    exportgraphics(f, nombre, 'Resolution', 110);
end

function [Kc,Ti,Td] = zn_pid(Ku, Tu)
    Kcs = Ku/1.6;  Tis = Tu/2;  Tds = Tu/8;                 % ZN: PID serie
    Kc = Kcs*(1+Tds/Tis);  Ti = Tis+Tds;  Td = Tis*Tds/(Tis+Tds);   % serie -> ideal
end

function M = lazo1(Gv, Gp, Kd, Gc, Gff)
    % Entradas [r d], salidas [y u]
    Gc = tf(Gc); Gc.InputName='e'; Gc.OutputName='ufb';
    Gff = tf(Gff); Gff.InputName='d'; Gff.OutputName='uff';
    Gv.InputName='u'; Gv.OutputName='v';  Gp.InputName='a'; Gp.OutputName='y';
    K = tf(Kd); K.InputName='d'; K.OutputName='dk';
    M = connect(Gc, Gff, Gv, Gp, K, sumblk('e = r - y'), sumblk('u = ufb + uff'), ...
                sumblk('a = v + dk'), {'r','d'}, {'y','u'});
end

function M = lazo2(Gv, G1, G2, Lp, Gd, Ld, Gce, Gci, Gff)
    % Gci vacío -> lazo simple sobre salida1. Entradas [r d], salidas [y1 u]
    G2d = G2; G2d.OutputDelay = Lp;  Gdd = Gd; Gdd.OutputDelay = Ld;
    Gv.InputName='u'; Gv.OutputName='v';  G1.InputName='a'; G1.OutputName='y2';
    G2d.InputName='y2'; G2d.OutputName='y1';  Gdd.InputName='d'; Gdd.OutputName='dd';
    Gff = tf(Gff); Gff.InputName='d'; Gff.OutputName='uff';
    Gce = tf(Gce); Gce.InputName='e1';
    if isempty(Gci)
        Gce.OutputName='ufb';  bloques = {Gce};
    else
        Gce.OutputName='r2';  Gci = tf(Gci); Gci.InputName='e2'; Gci.OutputName='ufb';
        bloques = {Gce, Gci, sumblk('e2 = r2 - y2')};
    end
    M = connect(bloques{:}, Gv, G1, G2d, Gdd, Gff, sumblk('e1 = r - y1'), ...
                sumblk('u = ufb + uff'), sumblk('a = v + dd'), {'r','d'}, {'y1','u'});
end

function metr(nombre, t, y, R, td)
    i = (t >= 1) & (t < td);  yy = y(i);  tt = t(i);
    Mp = max(0, (max(yy)-R)/R*100);  b = find(abs(yy-R) > 0.02*R, 1, 'last');
    j = t >= td;  dev = y(j) - R;
    fprintf('  %-12s Mp = %5.1f %%  ts(2%%) = %5.2f  pico ante d = %+.4f\n', nombre, Mp, tt(b)-1, dev(find(abs(dev)==max(abs(dev)),1)));
end
