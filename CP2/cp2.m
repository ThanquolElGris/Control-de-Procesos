%% CP-2: Control para plantas de primer orden
clear; clc; close all;

%% ===================== Ejercicio 1: modelo del mezclador =====================
x1 = 0.6; F2 = 1; V = 10;           % constantes (rho se cancela)
F1o = 3; x2o = 0.4;                 % punto de operación conocido
F3o = F1o + F2;                     % c) balance total en estado estable
x3o = (x1*F1o + x2o*F2)/F3o;        % c) balance parcial en estado estable
fprintf('F3o = %.2f cm3/min   x3o = %.4f\n', F3o, x3o);

% e) Linealización: V dx3/dt = x1 F1 + x2 F2 - x3 (F1 + F2)
%    10 dX3/dt + 4 X3 = (x1 - x3o) F1 + F2 X2 = 0.05 F1 + 1 X2
G_x3F1 = tf(x1 - x3o, [V F3o]);     % 0.05/(10s+4) = 0.0125/(2.5s+1)
G_x3x2 = tf(F2,       [V F3o]);     % 1/(10s+4)    = 0.25/(2.5s+1)
G_F3F1 = tf(1);                     % F3 = F1 + F2  ->  dF3 = dF1

% g) Escalones del 10 % en cada entrada
figure('Name','1g');
subplot(3,1,1); step(0.1*F1o*G_x3F1, 20); grid on; title('\Deltax3 ante +10% en F1 (\DeltaF1 = 0.3)');
subplot(3,1,2); step(0.1*F1o*G_F3F1, 20); grid on; title('\DeltaF3 ante +10% en F1');
subplot(3,1,3); step(0.1*x2o*G_x3x2, 20); grid on; title('\Deltax3 ante +10% en x2 (\Deltax2 = 0.04)');
fprintf('Dx3(inf) por F1: %.5f   por x2: %.5f   ts(2%%) = 4*2.5 = 10 min\n', ...
        dcgain(0.3*G_x3F1), dcgain(0.04*G_x3x2));

%% ===================== Ejercicio 2: control de x3 con F1 =====================
G  = tf(0.05, [10 4]);   % planta     K = 0.0125, T = 2.5
Gd = tf(1,    [10 4]);   % perturbación
K = 0.0125; T = 2.5;
r0 = 0.02; d0 = 0.01; umax = 2;

% Diseño A: PI por cancelación (Ti = T) con la mayor Kp que no satura.
% u(0+) = Kp*r0 y u(inf) = r0/K = 1.6  ->  Kp <= umax/r0 = 100
KpA = umax/r0;  TiA = T;
GcA = KpA*tf([TiA 1], [TiA 0]);
TlcA = T/(KpA*K);
fprintf('\nA) Kp = %.1f  Ti = %.2f  Tlc = %.2f min  ts ~ %.1f min\n', KpA, TiA, TlcA, 4*TlcA);

% Diseño B: PI con 2º orden deseado (zeta = 0.7) y prefiltro 1/(Ti s + 1)
z = 0.7; tss = 9.5;                 % tss mínimo que no satura ~ 9.34 min
wn  = 4/(z*tss);
KpB = (2*z*wn*T - 1)/K;
TiB = KpB*K/(T*wn^2);
GcB = KpB*tf([TiB 1], [TiB 0]);
Pf  = tf(1, [TiB 1]);
fprintf('B) wn = %.3f  Kp = %.2f  Ti = %.3f\n', wn, KpB, TiB);

t = 0:0.01:40;
figure('Name','2b');
disenos = {GcA, 1; GcB, Pf};
for i = 1:2
    Gc = disenos{i,1}; P = disenos{i,2};
    subplot(3,1,1); hold on; step(r0*P*feedback(Gc*G, 1), t);        % x3 / referencia
    subplot(3,1,2); hold on; step(r0*P*feedback(Gc, G), t);          % u  / referencia
    subplot(3,1,3); hold on; step(d0*Gd*feedback(1, Gc*G), t);       % x3 / perturbación
end
subplot(3,1,1); grid on; title('\Deltax3 ante escalón 0.02 en la referencia'); legend('A','B');
subplot(3,1,2); grid on; title('Acción de control \DeltaF1'); yline(umax,'r--'); legend('A','B');
subplot(3,1,3); grid on; title('\Deltax3 ante escalón 0.01 en la perturbación'); legend('A','B');
