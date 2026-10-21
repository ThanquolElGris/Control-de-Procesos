%% Certamen 1 Control de Procesos 2025: lazo único vs. mando subordinado
clear; clc; close all;
s = tf('s');

%% 1) Modelo (bloques del enunciado)
G1 = 4/(s+2);          % = 2/(0.5s+1)
G2 = 0.5/(2*s+1);      % -> med2
G3 = 2/(4*s+1);
Gs = 10/(s+10);        % sensor de la salida -> med1
Gp = 1/(3*s+2);        % camino de la perturbación
L3 = 1.5; Ls = 0.5; Lp = 1;
Gmy = G1*G2*G3*exp(-L3*s);        % man  -> salida
H   = Gs*exp(-Ls*s);              % salida -> med1
Gd  = Gp*exp(-Lp*s)*G3*exp(-L3*s);% pert -> salida
fprintf('Ganancias: man->salida = %.2f, pert->salida = %.2f\n', dcgain(G1*G2*G3), dcgain(Gp*G3));

%% 2) Lazo único (medición med1): Ku, Tu y PORT
Gm = Gmy*H;                                   % man -> med1 (retardo total 2 s)
[Gm_, ~, wu] = margin(Gm); Ku = Gm_; Tu = 2*pi/wu;
[K, T, L] = port(Gm);
fprintf('\nLazo único: Ku = %.4f  wu = %.4f  Tu = %.3f | PORT K = %.3f T = %.3f L = %.3f L/T = %.3f\n', Ku, wu, Tu, K, T, L, L/T);

% Sintonías (formas ideales Kc, Ti, Td)
p = struct();
p.ZN   = zn_ideal(Ku, Tu);
p.CC   = [T/(K*L)*(4/3 + L/T/4), L*(32+6*L/T)/(13+8*L/T), 4*L/(11+2*L/T)];
p.ITAE_ref  = itae_ref(K, T, L);
p.ITAE_pert = [1.357/K*(L/T)^-0.947, T/0.842*(L/T)^0.738, 0.381*T*(L/T)^0.995];
imprimir(p);
pA = p.ITAE_ref;                              % elegido para el lazo único
[PA, IA, DA] = deal(pA(1), pA(1)/pA(2), pA(1)*pA(3));
KbA = 1/sqrt(pA(2)*pA(3));
fprintf('=> Lazo único (ITAE-ref): P = %.4f  I = %.4f  D = %.4f  N = 10  Kb = %.4f\n', PA, IA, DA, KbA);

%% 3) Cascada: interno por MO sobre med2, externo por ITAE-ref sobre med1
Gi   = G1*G2;                                  % man -> med2 = 1/((0.5s+1)(2s+1)), sin retardo
Tu_i = 0.5;
Gc1  = minreal(1/(2*Tu_i*s*(Tu_i*s+1)*Gi))     % = (2s+1)/s = 2 + 1/s  (PI)
Gcl1 = minreal(feedback(Gc1*Gi, 1))            % = 2/(s^2+2s+2) ~ 1/(s+1)
Go   = Gcl1*G3*exp(-L3*s)*H;                   % lo que "ve" el externo (med2ref -> med1)
[Gm2, ~, wu2] = margin(Go); Ku2 = Gm2; Tu2 = 2*pi/wu2;
[K2, T2, L2] = port(Go);
fprintf('\nExterno: Ku = %.4f  Tu = %.3f | PORT K = %.3f T = %.3f L = %.3f L/T = %.3f\n', Ku2, Tu2, K2, T2, L2, L2/T2);
pB = itae_ref(K2, T2, L2);
[PB, IB, DB] = deal(pB(1), pB(1)/pB(2), pB(1)*pB(3));
KbB = 1/sqrt(pB(2)*pB(3));
fprintf('=> Externo (ITAE-ref): P = %.4f  I = %.4f  D = %.4f  N = 10  Kb = %.4f\n', PB, IB, DB, KbB);
fprintf('=> Interno (MO): P = 2  I = 1  Kb = 1/Ti = 0.5\n');

%% 4) Respuestas lineales (sin saturación). D sobre la medición:
%     Y/R = (P + I/s)*Gplanta/(1 + PID*Gplanta*H),   Y/D = Gd/(1 + PID*Gplanta*H)
N = 10;
CA  = pid(PA, IA, DA, 1/N);  CAr = pid(PA, IA);
CB  = pid(PB, IB, DB, 1/N);  CBr = pid(PB, IB);
TrA = CAr*feedback(Gmy, CA*H);                 TdA = Gd*feedback(1, CA*Gmy*H);
Gy2 = Gcl1*G3*exp(-L3*s);                      % referencia de med2 -> salida
TrB = CBr*feedback(Gy2, CB*H);                 TdB = Gd*feedback(1, CB*Gy2*H);
t = (0:0.01:80)';
yA = step(TrA, t); yB = step(TrB, t); dA = step(TdA, t); dB = step(TdB, t);
figure('Name', 'Certamen 1 2025 (lineal)');
subplot(2,1,1); plot(t, yA, t, yB); grid on; yline(1.1, ':'); legend('Lazo único', 'Cascada');
title('Escalón unitario en la referencia (D sobre la medición)');
subplot(2,1,2); plot(t, dA, t, dB); grid on; legend('Lazo único', 'Cascada');
title('Escalón unitario en la perturbación'); xlabel('t [s]');
SA = stepinfo(yA, t, 1, 'RiseTimeLimits', [0.1 0.9]); SB = stepinfo(yB, t, 1, 'RiseTimeLimits', [0.1 0.9]);
fprintf('\nLineal: único  Mp = %.1f%%  tr = %.2f s  ts = %.2f s | pico pert = %.3f\n', SA.Overshoot, SA.RiseTime, SA.SettlingTime, max(dA));
fprintf('Lineal: cascada Mp = %.1f%%  tr = %.2f s  ts = %.2f s | pico pert = %.3f\n', SB.Overshoot, SB.RiseTime, SB.SettlingTime, max(dB));
% La saturación ±2 y el anti-windup se prueban en Simulink (ver README).

%% ---------- funciones auxiliares ----------
function [K, T, L] = port(G)
    t = (0:0.001:80)'; y = step(G, t); K = y(end);
    t28 = t(find(y >= 0.283*K, 1)); t63 = t(find(y >= 0.632*K, 1));
    T = 1.5*(t63 - t28); L = t63 - T;
end
function p = zn_ideal(Ku, Tu)                  % ZN serie -> ideal
    Kc = Ku/1.6; Ti = Tu/2; Td = Tu/8;
    p = [Kc*(1+Td/Ti), Ti+Td, Ti*Td/(Ti+Td)];
end
function p = itae_ref(K, T, L)                 % Rovira, PID ideal
    r = L/T; p = [0.965/K*r^-0.855, T/(0.796-0.147*r), 0.308*T*r^0.929];
end
function imprimir(p)
    f = fieldnames(p);
    for i = 1:numel(f)
        q = p.(f{i});
        fprintf('  %-9s Kc = %.4f  Ti = %.4f  Td = %.4f  ->  P = %.4f  I = %.4f  D = %.4f\n', ...
                f{i}, q, q(1), q(1)/q(2), q(1)*q(3));
    end
end
