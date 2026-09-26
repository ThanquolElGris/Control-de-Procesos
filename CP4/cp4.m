%% CP-4: Control de plantas
clear; clc; close all;
s = tf('s');
znPID = @(Ku,Tu) [Ku/1.6, Tu/2, Tu/8];              % ZN lazo cerrado, PID serie [K'c T'i T'd]

%% ======================== EJERCICIO 1 ========================
Gv = 2/(s+2);  G1 = 5/(2*s+1);  G2 = 3/(5*s+1);  L = 1;

% ---- a1) Ku y Tu (y modelo PORT) de med2/man ----
Ga = Gv*G1*G2;  Ga.InputDelay = L;
[Gm,~,Wcg] = margin(Ga);  Ku = Gm;  Tu = 2*pi/Wcg;
fprintf('a1) Ku = %.4f  wu = %.4f rad/s  Tu = %.4f s\n', Ku, Wcg, Tu);
t = 0:0.001:60;  y = step(Ga, t);  K = y(end);
t28 = t(find(y >= 0.283*K,1));  t63 = t(find(y >= 0.632*K,1));
T = 1.5*(t63-t28);  Lp = t63-T;
fprintf('    PORT: K = %.2f  t28 = %.3f  t63 = %.3f  T = %.3f  L = %.3f\n', K, t28, t63, T, Lp);

% ---- a2) ZN -> ideal -> paralelo ----
p = znPID(Ku,Tu);                                         % serie
Kc = p(1)*(1+p(3)/p(2));  Ti = p(2)+p(3);  Td = p(2)*p(3)/(p(2)+p(3));
P1 = Kc;  I1 = Kc/Ti;  D1 = Kc*Td;
fprintf('a2) serie: %.4f %.4f %.4f | ideal: Kc=%.4f Ti=%.4f Td=%.4f | P=%.4f I=%.4f D=%.4f\n', p, Kc, Ti, Td, P1, I1, D1);

% ---- b1-b2) Controlador interno por MO sobre med1/man = Gv*G1 (Tu = 0.5) ----
Gi  = Gv*G1;
Gc1 = minreal(1/(2*0.5*s*(0.5*s+1)*Gi))                  % = 0.4 + 0.2/s
Gcl1 = minreal(feedback(Gc1*Gi, 1))                      % = 2/(s^2+2s+2)

% ---- b3-b4) PID externo por ZN sobre med2/ref1 = Gcl1*G2*e^-s ----
Gb = Gcl1*G2;  Gb.InputDelay = L;
[Gm,~,Wcg] = margin(Gb);  Ku2 = Gm;  Tu2 = 2*pi/Wcg;
p = znPID(Ku2,Tu2);
Kc = p(1)*(1+p(3)/p(2));  Ti = p(2)+p(3);  Td = p(2)*p(3)/(p(2)+p(3));
P2 = Kc;  I2 = Kc/Ti;  D2 = Kc*Td;
fprintf('b3) Ku = %.4f  Tu = %.4f | b4) P = %.4f  I = %.4f  D = %.4f\n', Ku2, Tu2, P2, I2, D2);

% Las respuestas a3 y b5 se simulan en Simulink (ver README): escalón en la referencia en t = 1
% y escalón en dist en t = 60. Graficar: (1) referencia y med2, (2) u, (3) error.

%% ======================== EJERCICIO 2 ========================
Gv = 3/(s+3);  G1 = 3/(2*s+1);  G2 = 4/s;
Gp = Gv*G1*G2;  Tu = 1/3;
Gcms = minreal((4*Tu*s+1)/(8*Tu^2*s^2*(Tu*s+1)*Gp))     % a) MS lazo único: 0.3125 + 0.09375/s + 0.25 s
Gcmo = minreal(1/(2*Tu*s*(Tu*s+1)*Gp))                   %    MO lazo único: 0.125 + 0.25 s
Gi = Gv*G1;
Gcin = minreal(1/(2*Tu*s*(Tu*s+1)*Gi))                   % b1) MO interno: 1 + 0.5/s
Tu2 = 2/3;  Go = G2/(Tu2*s+1);                           % lazo interno ~ 1/(2Tu s + 1)
Gcexms = minreal((4*Tu2*s+1)/(8*Tu2^2*s^2*(Tu2*s+1)*Go)) % b3) MS externo: 0.1875 + 0.07031/s
Gcexmo = minreal(1/(2*Tu2*s*(Tu2*s+1)*Go))               %     MO externo: 0.1875 (P)

Gin = feedback(Gcin*Gi, 1);
figure; hold on; grid on
step(feedback(Gcms*Gp,1), 12); step(feedback(Gcmo*Gp,1), 12);
step(feedback(Gcexms*Gin*G2,1), 12); step(feedback(Gcexmo*Gin*G2,1), 12);
legend('MS','MO','cascada MS MO','cascada MO MO'); title('Ej. 2: escalón en la referencia')

% Perturbación (entra en med1, antes de G2):
%   lazo único:  Y/D = G2/(1 + Gc*Gv*G1*G2)
%   cascada:     m1 = d - Gi*Gcin*(1 + Gcex*G2)*m1  ->  Y/D = G2/(1 + Gi*Gcin*(1 + Gcex*G2))
figure; hold on; grid on
step(feedback(G2, Gcms*Gi), 30);  step(feedback(G2, Gcmo*Gi), 30);
step(G2*feedback(1, Gi*Gcin*(1 + Gcexms*G2)), 30);  step(G2*feedback(1, Gi*Gcin*(1 + Gcexmo*G2)), 30);
legend('MS','MO','cascada MS MO','cascada MO MO'); title('Ej. 2: escalón en la perturbación')
