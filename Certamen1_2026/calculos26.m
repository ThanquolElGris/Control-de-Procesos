%% CALCULOS26  Certamen 1 2026: identificación y sintonía (sin toolboxes)
% Planta: man -> G1 -> (+pert) -> G2 -> med2 -> G3 -> e^-1.5s -> salida -> Gs -> e^-0.5s -> med1
%   G1 = 4/(s+2) = 2/(0.5s+1)   G2 = 0.5/(2s+1)   G3 = 2/(4s+1)   Gs = 10/(s+10) = 1/(0.1s+1)
%   Gp = 1/(3s+2) = 0.5/(1.5s+1) con retardo 1 s (la perturbación entra ENTRE G1 y G2)
clear; clc;

%% 1) Ganancias y mando en estado estable
Kman = 2*0.5*2;            % man  -> salida
Kpert = 0.5*0.5*2;         % pert -> salida
fprintf('Ganancia man->salida = %.2f, pert->salida = %.2f\n', Kman, Kpert);
fprintf('man_ss ante ref. unitaria = %.2f; ante pert. unitaria = %.2f  (límite ±2)\n', 1/Kman, -Kpert/Kman);

%% 2) Lazo único (man -> med1): Ku, Tu (condición de fase) y PORT
fase1 = @(w) -atan(0.5*w) - atan(2*w) - atan(4*w) - atan(0.1*w) - 2*w;       % retardo total 2 s
mag1  = @(w) 2 ./ sqrt((1+(0.5*w).^2).*(1+(2*w).^2).*(1+(4*w).^2).*(1+(0.1*w).^2));
wu = fzero(@(w) fase1(w) + pi, 0.5); Ku = 1/mag1(wu); Tu = 2*pi/wu;
[K, T, L] = port26([0.5 2 4 0.1], 2, 2);
fprintf('\nLazo único: Ku = %.4f  wu = %.4f  Tu = %.3f | PORT K = %.3f T = %.3f L = %.3f L/T = %.3f\n', Ku, wu, Tu, K, T, L, L/T);
A = reglas26(Ku, Tu, K, T, L);

%% 3) Cascada. Interno (man -> med2 = 1/((0.5s+1)(2s+1)), sin retardo) por módulo óptimo
Tui = 0.5;  Ki = 1;                                                   % se compensa la constante de 2 s
% Gc1 = (2s+1)/(2*Tui*Ki*s)  ->  PI con P = T2/(2*Tui*Ki) = 2, I = 1/(2*Tui*Ki) = 1
P1 = 2/(2*Tui*Ki); I1 = 1/(2*Tui*Ki);
fprintf('\nInterno (MO): Gc1 = (2s+1)/s  ->  P = %.3f  I = %.3f  (Kb = 1/Ti = %.3f)\n', P1, I1, I1/P1);
fprintf('Lazo interno cerrado = 2/(s^2+2s+2) ~ 1/(s+1)\n');
% Externo: 2/(s^2+2s+2) * G3 * Gs * e^-2s
fase2 = @(w) -atan2(2*w, 2 - w.^2) - atan(4*w) - atan(0.1*w) - 2*w;
mag2  = @(w) 2*2 ./ sqrt(((2 - w.^2).^2 + 4*w.^2).*(1+(4*w).^2).*(1+(0.1*w).^2));
wu2 = fzero(@(w) fase2(w) + pi, 0.6); Ku2 = 1/mag2(wu2); Tu2 = 2*pi/wu2;
[K2, T2, L2] = port26([4 0.1], 2, 2, true);
fprintf('Externo: Ku = %.4f  wu = %.4f  Tu = %.3f | PORT K = %.3f T = %.3f L = %.3f L/T = %.3f\n', Ku2, wu2, Tu2, K2, T2, L2, L2/T2);
B = reglas26(Ku2, Tu2, K2, T2, L2);

%% 4) Diseños elegidos (ver README)
fprintf('\n=> Lazo único: PID ITAE-perturbación, b = 0, c = 0:  P = %.4f  I = %.4f  D = %.4f  Kb = %.4f\n', ...
        A.ITAE_pert(1), A.ITAE_pert(1)/A.ITAE_pert(2), A.ITAE_pert(1)*A.ITAE_pert(3), 1/sqrt(A.ITAE_pert(2)*A.ITAE_pert(3)));
fprintf('=> Cascada: interno PI P = 2, I = 1; externo PID ITAE-referencia, b = 1, c = 0:  P = %.4f  I = %.4f  D = %.4f  Kb = %.4f\n', ...
        B.ITAE_ref(1), B.ITAE_ref(1)/B.ITAE_ref(2), B.ITAE_ref(1)*B.ITAE_ref(3), 1/sqrt(B.ITAE_ref(2)*B.ITAE_ref(3)));
save('sintonias26.mat', 'A', 'B', 'P1', 'I1');
% Funciones auxiliares: reglas26.m y port26.m
