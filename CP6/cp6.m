%% CP-6: Predictor de Smith, compensador de respuesta inversa y control inferencial
% Simulación con connect + lsim discretizado (ZOH exacto para escalones). Los modelos
% Simulink de los ej. 3 y 5 se construyen con cp6_simulink.m.
clear; clc; close all;
s = tf('s');
set(groot, 'defaultFigureVisible', 'off');
warning('off', 'all');                               % avisos de margin/c2d con retardos internos
simu = @(M, u, t) lsim(c2d(M, t(2)-t(1), 'zoh'), u, t);

%% ======================== EJERCICIO 1: calentador PORT ========================
K = 5;  T = 1;  L = 0.8;                             % °C/V, min, min ; u en [0, 10] V
P = K/(T*s+1);
G = P;  G.OutputDelay = L;
fprintf('Ej1: L/T = %.2f\n', L/T);

% ---- a) PI por ZN ----
[Ku,~,wu] = margin(G);  Tu = 2*pi/wu;
Kc_zn = Ku/2.2;  Ti_zn = Tu/1.2;
fprintf('a) Ku = %.4f  wu = %.4f rad/min  Tu = %.4f min  ->  Kc = %.4f  Ti = %.4f min\n', Ku, wu, Tu, Kc_zn, Ti_zn);
C_zn = Kc_zn*(1 + 1/(Ti_zn*s));

% ---- b) PI por cancelación (Ti = T) + predictor de Smith ----
% u(0+) = Kp*Δr  ->  Kp = 10 V / 2 °C = 5  (máximo sin saturar)
Kp = 10/2;  Ti = T;
C_sp = Kp*(Ti*s+1)/(Ti*s);
fprintf('b) Kp = %.2f  Ti = %.2f  ->  lazo sin retardo: Y/R = %s  (T_LC = %.3f min)\n', Kp, Ti, ...
        'K*Kp/(T s + K*Kp)', T/(K*Kp));

% Experimento: r = 2 °C en t = 1 min, perturbación 0.1 V (a la entrada) en t = 10 min
t = (0:0.002:20)';  r = 2*(t >= 1);  d = 0.1*(t >= 10);
Mzn = lazo_simple(C_zn, G);
Msp = smith(C_sp, G, P, L, 1);
yzn = simu(Mzn, [r d], t);  ysp = simu(Msp, [r d], t);
f = figure('Position',[100 100 800 650]);
subplot(2,1,1); plot(t, yzn(:,1), t, ysp(:,1), t, r, 'k--'); grid on; ylabel('T [°C]');
legend('PI por ZN','PI cancelación + Smith','referencia','Location','southeast');
title('CP6 Ej. 1: r = 2 °C en t = 1, d = 0.1 V en t = 10');
subplot(2,1,2); plot(t, yzn(:,2), t, ysp(:,2)); grid on; ylabel('u [V]'); xlabel('t [min]');
guardar(f, 'ej1_ab.png');
metr('PI ZN', t, yzn(:,1), 2, 10);  metr('Smith', t, ysp(:,1), 2, 10);
fprintf('   u máx: ZN = %.3f V, Smith = %.3f V\n', max(yzn(:,2)), max(ysp(:,2)));

% ---- b) Robustez: ±10 % en K, T y L (uno a la vez); el predictor mantiene el modelo nominal ----
tit = {'K','T','L'};
for Kpr = [Kp 1]                                     % Kp = 5 (pedido) y Kp = 1 (T_LC = 0.2 min, más robusto)
    Cr = Kpr*(Ti*s+1)/(Ti*s);
    f = figure('Position',[100 100 900 650]);
    fprintf('b) robustez con Kp = %g (predictor nominal, planta modificada):\n', Kpr);
    for j = 1:3
        subplot(3,1,j); hold on;
        for fac = [0.9 1 1.1]
            p = [K T L];  p(j) = p(j)*fac;
            Gr = tf(p(1),[p(2) 1]);  Gr.OutputDelay = p(3);
            y = simu(smith(Cr, Gr, P, L, 1), [r 0*d], t);
            plot(t, y(:,1));
            fprintf('   %s x %.1f: Mp = %6.1f %%, y(20) = %.3f\n', tit{j}, fac, max(0,(max(y(:,1))-2)/2*100), y(end,1));
        end
        plot(t, r, 'k--'); grid on; ylim([0 3.5]); ylabel('T [°C]');
        legend([tit{j} ' -10 %'], 'nominal', [tit{j} ' +10 %'], 'Location','southeast');
    end
    xlabel('t [min]'); sgtitle(sprintf('Ej. 1 b): Smith con error de modelo del 10 %%, K_p = %g', Kpr));
    guardar(f, sprintf('ej1_robustez_Kp%g.png', Kpr));
end

% ---- c) Predictor de Smith filtrado: Fr = (beta s + 1)/(Tf s + 1) ----
% Y/D = P e^{-Ls} [1 - C P e^{-Ls} Fr/(1+CP)] ; se anula el polo lento s = -1/T si
% Fr(-1/T) = (1 + CP)/(CP e^{-Ls}) evaluado en s = -1/T
s0 = -1/T;  CP0 = K*Kp/(T*s0);                      % C*P = K Kp/(T s) por la cancelación
Fr0 = (1 + CP0)/(CP0*exp(-L*s0));
Tf = L/2;  beta = (1 - Fr0*(1 + Tf*s0))/(-s0);       % (beta*s0+1)/(Tf*s0+1) = Fr0
Fr = (beta*s+1)/(Tf*s+1);
fprintf('c) Fr(-1) = %.4f  ->  Fr = (%.4f s + 1)/(%.2f s + 1)\n', Fr0, beta, Tf);
Mf = smith(C_sp, G, P, L, Fr);
yf = simu(Mf, [r d], t);
f = figure('Position',[100 100 800 600]);
subplot(2,1,1); plot(t, ysp(:,1), t, yf(:,1), '--'); grid on; xlim([9 20]); ylabel('T [°C]');
legend('Smith','Smith filtrado'); title('Ej. 1 c): perturbación de 0.1 V en t = 10 min');
subplot(2,1,2); plot(t, ysp(:,2), t, yf(:,2), '--'); grid on; xlim([9 20]); ylabel('u [V]'); xlabel('t [min]');
guardar(f, 'ej1_filtro.png');
nom = {'Smith', 'Smith filtrado'};  ys = {ysp(:,1), yf(:,1)};
for k = 1:2
    j = t >= 10;  dev = ys{k}(j) - 2;  b = find(abs(dev) > 0.02*max(abs(dev)), 1, 'last');
    fprintf('   %-15s pico = %.4f °C   se recupera (2%% del pico) en %.2f min\n', nom{k}, max(dev), b*(t(2)-t(1)));
end

%% ======================== EJERCICIO 2: respuesta inversa con válvula ========================
Gv = 2/(0.5*s+1);  G1 = 2/(3*s+1);  G2 = 1/(s+1);
Gp = (-s+1)/((3*s+1)*(s+1));                        % = G1 - G2
assert(norm(Gp - (G1 - G2), inf) < 1e-9)
G  = Gv*Gp

% ---- a) PI por ZN ----
[Ku,~,wu] = margin(G);  Tu = 2*pi/wu;
C_zn = (Ku/2.2)*(1 + 1/((Tu/1.2)*s));
fprintf('\nEj2 a) Ku = %.4f  Tu = %.4f  ->  Kc = %.4f  Ti = %.4f\n', Ku, Tu, Ku/2.2, Tu/1.2);

% ---- b) Compensador: y* = y + Kcomp*Gv*(1/(s+1) - 1/(3s+1))*u ----
K1 = 2; t1 = 3; K2 = 1; t2 = 1;
Kcomp = (K1*t2 - K2*t1)/(t2 - t1);                   % = 0.5
Gcomp = Gv*s/((3*s+1)*(s+1))                        % = 0.5*Gv*(1/(s+1) - 1/(3s+1)) = 2s/((0.5s+1)(3s+1)(s+1))
assert(norm(Gcomp - Kcomp*Gv*(1/(t2*s+1) - 1/(t1*s+1)), inf) < 1e-9)
Gstar = Gv/((3*s+1)*(s+1))                           % lo que enfrenta el controlador
assert(norm(Gstar - (G + Gcomp), inf) < 1e-9)

% ---- c) Módulo óptimo sobre Gstar (Tu = 0.5): PID ----
Gc_mo = (3*s+1)*(s+1)/(2*s)                         % = 1/(2*0.5*s*(0.5s+1)*Gstar) = 2 + 0.5/s + 1.5 s
C_mo  = pid(2, 0.5, 1.5, 0.05);                      % con filtro derivativo para implementarlo

t = (0:0.005:60)';  r = 1*(t >= 1);  d = 1*(t >= 30);
Ma = lazo_simple(C_zn, G, Gv);                       % la perturbación entra antes de la válvula
Mc = lazo_comp(C_mo, G, Gcomp, Gv);
ya = simu(Ma, [r d], t);  yc = simu(Mc, [r d], t);
f = figure('Position',[100 100 800 650]);
subplot(2,1,1); plot(t, ya(:,1), t, yc(:,1), t, r, 'k--'); grid on; ylabel('h');
legend('PI por ZN','compensador + MO (PID)','referencia','Location','southeast');
title('CP6 Ej. 2: r = 1 en t = 1, d = 1 en t = 30');
subplot(2,1,2); plot(t, ya(:,2), t, yc(:,2)); grid on; ylabel('u'); xlabel('t [s]'); ylim([-6 6]);
guardar(f, 'ej2.png');
metr('PI ZN', t, ya(:,1), 1, 30);  metr('comp + MO', t, yc(:,1), 1, 30);

% ---- d) Conjunto controlador-compensador: u = Gc/(1 + Gc*Gcomp) * (r - y) ----
Geq = zpk(minreal(feedback(Gc_mo, Gcomp), 1e-6))     % = 1.5 (s+2)(s+1)(s+0.333)/(s (s+4))
Geq_f = zpk(minreal(feedback(tf(C_mo), Gcomp), 1e-6)) % con el PID filtrado (propia)

%% ======================== EJERCICIO 3: Gv = 2/(2s+2), Gp = (-5s+1)/(4s+1) ========================
Gv = 2/(2*s+2);  Gp = (-5*s+1)/(4*s+1);  G = minreal(Gv*Gp);
% Gp = 2.25/(4s+1) - 1.25  -> K1 = 2.25, t1 = 4, K2 = 1.25, t2 = 0
Kcomp = (2.25*0 - 1.25*4)/(0 - 4);                    % = 1.25
Gcomp = 5*s/((s+1)*(4*s+1))                         % = 1.25*Gv*(1 - 1/(4s+1))
assert(norm(Gcomp - Kcomp*Gv*(1 - 1/(4*s+1)), inf) < 1e-9)
Gstar = 1/((s+1)*(4*s+1));                            % planta compensada
assert(norm(Gstar - (G + Gcomp), inf) < 1e-9)
% Con MO (a = 2) y*/r tiene 4.3 %, pero med/r = (1-5s)/(2s^2+2s+1): el cero SPD sube el sobrepaso a ~10 %.
% Se busca el menor a (ζ = sqrt(a)/2) que deja med con Mp <= 4 %:
for a = 2:0.1:4
    if stepinfo((1-5*s)/(a*s^2+a*s+1)).Overshoot <= 4, break; end
end
fprintf('\nEj3: a = %.1f  (zeta = %.3f)\n', a, sqrt(a)/2);
Gc3  = (4*s+1)/(a*s);                                 % 1/(a*Tu*s*(Tu s+1)*Gstar), Tu = 1
Gc3mo = (4*s+1)/(2*s);                                % MO clásico (a = 2), para comparar
t = (0:0.005:30)';  r = (t >= 1);
y3  = simu(lazo_comp(Gc3, G, Gcomp, 1), [r 0*r], t);
y3mo = simu(lazo_comp(Gc3mo, G, Gcomp, 1), [r 0*r], t);
Gc0 = (4*s+1)/(2*s*(-5*s+1));                        % MO "ingenuo" sobre G sin compensar: 1/(2s(s+1)G)
fprintf('\nEj3: MO directo sobre la planta (sin compensar) tiene polos: %s\n', mat2str(pole(Gc0)',3));
f = figure('Position',[100 100 800 600]);
subplot(2,1,1); plot(t, y3mo(:,1), t, y3(:,1), t, r, 'k--'); grid on; ylabel('med');
legend('MO (a = 2)', sprintf('a = %.1f', a), 'referencia', 'Location','southeast');
title('CP6 Ej. 3: compensador de RI + PI, escalón unitario en la referencia');
subplot(2,1,2); plot(t, y3mo(:,2), t, y3(:,2)); grid on; ylabel('man'); xlabel('t');
guardar(f, 'ej3.png');
metr('Ej3 a=2', t, y3mo(:,1), 1, 1e9);  metr('Ej3 a=2.5', t, y3(:,1), 1, 1e9);
fprintf('   subpaso inicial = %.4f\n', min(y3(:,1)));

%% ======================== EJERCICIO 4: k1/(Ts+1) - k2/(s(Ts+1)) ========================
f = figure('Position',[100 100 800 600]);  t = (0:0.005:40)';  r = (t >= 1);  par = [4 2 3; 2 4 3];
for k = 1:2
    k1 = par(k,1); k2 = par(k,2); T = par(k,3);
    G = minreal(k1/(T*s+1) - k2/(s*(T*s+1)));         % (k1 s - k2)/(s(Ts+1)), cero en s = k2/k1
    Gcomp = -k1/(T*s+1);                              % y* = y + Gcomp*u = -k2/(s(Ts+1)) u
    Gstar = minreal(G + Gcomp);
    Gc4 = -1/(2*T*k2);                                % MO con Tu = T: P (la planta ya integra)
    fprintf('\nEj4 (k1=%g, k2=%g, T=%g): cero de G en s = %.3f | Gcomp = -%g/(%gs+1) | Gc = %.5f\n', ...
            k1, k2, T, zero(G), k1, T, Gc4);
    Gstar
    y = simu(lazo_comp(Gc4, G, Gcomp, 1), [r 0*r], t);
    subplot(2,1,k); plot(t, y(:,1), t, r, 'k--'); grid on;
    title(sprintf('k_1 = %g, k_2 = %g, T = %g  (cero en s = +%.2f)', k1, k2, T, k2/k1)); ylabel('y');
    metr(sprintf('Ej4 caso %d', k), t, y(:,1), 1, 1e9);
    fprintf('   subpaso inicial = %.4f\n', min(y(:,1)));
end
xlabel('t'); guardar(f, 'ej4.png');

%% ======================== EJERCICIO 5: control inferencial ========================
Gd1 = 0.9/(70*s+1);  Gd1.OutputDelay = 5;            % XD <- XF
Gp1 = 1.2/(30*s+1);  Gp1.OutputDelay = 10;           % XD <- R
Gd2 = 0.2/(60*s+1);  Gd2.OutputDelay = 5;            % T  <- XF
Gp2 = 1/(20*s+1);                                    % T  <- R
Gr  = 4.5*(60*s+1)/(70*s+1);                         % Gd1/Gd2 (los retardos se cancelan)
fprintf('\nEj5: Gd1/Gd2 = 4.5(60s+1)/(70s+1)\n');
% El lazo ve Gp1 (PORT: K = 1.2, T = 30, L = 10, L/T = 0.33)
% Referencia: ZN de lazo abierto (serie 3, 20, 5) -> ideal
Kcs = 1.2*30/(1.2*10);  Tis = 2*10;  Tds = 0.5*10;
Kz = Kcs*(1+Tds/Tis);  Tiz = Tis+Tds;  Tdz = Tis*Tds/(Tis+Tds);
fprintf('   ZN: serie Kc = %.3f Ti = %.1f Td = %.1f | ideal Kc = %.3f Ti = %.1f Td = %.1f\n', Kcs, Tis, Tds, Kz, Tiz, Tdz);
% Elegido: ITAE para cambios en la referencia (Conf. 2), PID ideal
Kp1 = 1.2; tau = 30; L5 = 10; q = L5/tau;
Kc = 0.965/Kp1*q^-0.855;  Ti = tau/(0.796 - 0.147*q);  Td = 0.308*tau*q^0.929;
fprintf('   ITAE ref: Kc = %.3f  Ti = %.2f  Td = %.2f\n', Kc, Ti, Td);
C5 = pidstd(Kc, Ti, Td, 10);  C5zn = pidstd(Kz, Tiz, Tdz, 10);

t = (0:0.05:800)';  r = (t >= 10);  xf = (t >= 400);
yz = simu(inferencial(C5zn, Gp1, Gd1, Gp2, Gd2, Gp1, Gp2, Gr), [r xf], t);
yi = simu(inferencial(C5, Gp1, Gd1, Gp2, Gd2, Gp1, Gp2, Gr), [r xf], t);
f = figure('Position',[100 100 800 350]);
plot(t, yz(:,1), t, yi(:,1), t, r, 'k:'); grid on; xlim([0 300]); legend('PID ZN','PID ITAE (referencia)','referencia');
xlabel('t [min]'); ylabel('X_D'); title('Ej. 5 b): elección de la sintonía (modelo exacto)');
guardar(f, 'ej5_sintonia.png');
metr('Ej5 ZN', t, yz(:,1), 1, 400);  metr('Ej5 ITAE', t, yi(:,1), 1, 400);
f = figure('Position',[100 100 900 750]);
casos = {'modelo exacto', 'planta: Gd1 con +20 % de ganancia', 'planta: Gp2 y Gd2 con +20 % de ganancia'};
for k = 1:3
    pGd1 = Gd1; pGd2 = Gd2; pGp2 = Gp2;
    if k == 2, pGd1 = 1.2*Gd1; end
    if k == 3, pGd2 = 1.2*Gd2; pGp2 = 1.2*Gp2; end
    M = inferencial(C5, Gp1, pGd1, pGp2, pGd2, Gp1, Gp2, Gr);
    y = simu(M, [r xf], t);
    subplot(3,1,k); plot(t, y(:,1), t, y(:,2), '--', t, r, 'k:'); grid on; ylabel('X_D');
    title(casos{k}); legend('X_D real','X_D inferida','referencia','Location','southeast');
    fprintf('   %-40s XD(400) = %.4f  XD(800) = %.4f  XDest(800) = %.4f\n', casos{k}, y(t==400,1), y(end,1), y(end,2));
end
xlabel('t [min]'); sgtitle('CP6 Ej. 5: control inferencial (r = 1 en t = 10, X_F = 1 en t = 400)');
guardar(f, 'ej5.png');

%% ======================== funciones ========================
function guardar(f, nombre)
    try, theme(f, 'light'); end
    exportgraphics(f, nombre, 'Resolution', 110);
end

function M = lazo_simple(C, G, ~)
    % Entradas [r d], salidas [y u]. d se suma a u a la entrada de G (antes de la válvula)
    C = tf(C); C.InputName = 'e'; C.OutputName = 'u';
    G.InputName = 'a'; G.OutputName = 'y';
    M = connect(C, G, sumblk('e = r - y'), sumblk('a = u + d'), {'r','d'}, {'y','u'});
end

function M = smith(C, G, P, L, Fr)
    % Predictor de Smith (filtrado si Fr ~= 1): e = r - [P u + Fr (y - P e^{-Ls} u)]
    C = tf(C); C.InputName = 'e'; C.OutputName = 'u';
    G.InputName = 'a'; G.OutputName = 'y';
    Pm = P; Pm.InputName = 'u'; Pm.OutputName = 'ym';
    Pd = P; Pd.OutputDelay = L; Pd.InputName = 'u'; Pd.OutputName = 'ymd';
    F = tf(Fr); F.InputName = 'ep'; F.OutputName = 'epf';
    M = connect(C, G, Pm, Pd, F, sumblk('a = u + d'), sumblk('ep = y - ymd'), ...
                sumblk('e = r - ym - epf'), {'r','d'}, {'y','u'});
end

function M = lazo_comp(C, G, Gcomp, ~)
    % Compensador de respuesta inversa: e = r - (y + Gcomp u). d se suma a u (antes de la válvula)
    C = tf(C); C.InputName = 'e'; C.OutputName = 'u';
    G.InputName = 'a'; G.OutputName = 'y';
    Gcomp = tf(Gcomp); Gcomp.InputName = 'u'; Gcomp.OutputName = 'yc';
    M = connect(C, G, Gcomp, sumblk('a = u + d'), sumblk('e = r - y - yc'), {'r','d'}, {'y','u'});
end

function M = inferencial(C, Gp1, Gd1, Gp2, Gd2, Gp1m, Gp2m, Gr)
    % Planta (Gp1, Gd1, Gp2, Gd2) y estimador con modelo nominal (Gp1m, Gp2m, Gr = Gd1/Gd2).
    % Entradas [r xf], salidas [xd xd_est]
    C = tf(C); C.InputName = 'e'; C.OutputName = 'R';
    Gp1.InputName='R'; Gp1.OutputName='x1';  Gd1.InputName='xf'; Gd1.OutputName='x2';
    Gp2.InputName='R'; Gp2.OutputName='T1';  Gd2.InputName='xf'; Gd2.OutputName='T2';
    E1 = Gp1m - Gr*Gp2m;  E1.InputName = 'R';  E1.OutputName = 'z1';
    E2 = Gr;              E2.InputName = 'T';  E2.OutputName = 'z2';
    M = connect(C, Gp1, Gd1, Gp2, Gd2, E1, E2, sumblk('xd = x1 + x2'), sumblk('T = T1 + T2'), ...
                sumblk('xdest = z1 + z2'), sumblk('e = r - xdest'), {'r','xf'}, {'xd','xdest'});
end

function metr(nombre, t, y, R, td)
    i = (t >= 1) & (t < td);  yy = y(i);  tt = t(i);
    Mp = max(0, (max(yy)-R)/R*100);  b = find(abs(yy-R) > 0.02*R, 1, 'last');
    fprintf('  %-12s Mp = %5.1f %%  ts(2%%) = %6.2f', nombre, Mp, tt(b)-1);
    if td < t(end)
        j = t >= td;  dev = y(j) - R;
        fprintf('  pico ante d = %+.4f', dev(find(abs(dev)==max(abs(dev)),1)));
    end
    fprintf('\n');
end
