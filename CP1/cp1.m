%% CP-1: Ajuste de controladores (ZN, CC, CI)
% Ejemplo 6-1.1 Smith & Corripio: control de temperatura de tanque agitado.
clear; clc; close all;

%% Paso 1: Modelos
Gv = tf(1.652, [0.2 1]);                              % Válvula: W / M
Gs = tf(1.183, conv([8.34 1], [0.502 1]));            % Temperatura / flujo de vapor
Gf = tf(-3.34*[0.524 1], conv([8.34 1], [0.502 1]));  % Temperatura / flujo de alimentación
H  = tf(1, [0.75 1]);                                 % Sensor-transmisor
G1 = Gv*Gs*H;                                         % C/M
G2 = Gf*H;                                            % C/F

%% Paso 2: Ku y Tu (ganancia y período críticos)
[Gm, Pm, Wcg, Wcp] = margin(G1);
Ku = Gm;              % en veces (no en dB)
Tu = 2*pi/Wcg;
fprintf('Ku = %.4f   wu = %.4f rad/min   Tu = %.4f min\n', Ku, Wcg, Tu);

%% Paso 3: Modelo PORT (K, T, L) desde la respuesta al paso en lazo abierto
t = 0:0.001:60;
y = step(G1, t);
K   = y(end);
t28 = t(find(y >= 0.283*K, 1));
t63 = t(find(y >= 0.632*K, 1));
T = 1.5*(t63 - t28);
L = t63 - T;
r = L/T;
fprintf('K = %.4f  t28 = %.3f  t63 = %.3f  T = %.3f  L = %.3f  L/T = %.3f\n', ...
        K, t28, t63, T, L, r);
Gport = tf(K, [T 1], 'InputDelay', L);
figure; step(G1, Gport, 40); legend('Planta G1', 'PORT'); grid on;
title('Respuesta al paso en lazo abierto');

%% Paso 4: Sintonías [Kc Ti Td]
P = struct();
P.ZN_Ku    = [Ku/1.6,        Tu/2,  Tu/8];               % ZN lazo cerrado (serie)
P.ZN_PORT  = [1.2*T/(K*L),   2*L,   0.5*L];              % ZN lazo abierto (serie)
P.CC       = [T/(K*L)*(4/3 + r/4), L*(32+6*r)/(13+8*r), 4*L/(11+2*r)];  % Cohen-Coon (ideal)
P.IAE_ref  = [1.086/K*r^-0.869, T/(0.740-0.130*r), 0.348*T*r^0.914];    % Rovira (ideal)
P.IAE_pert = [1.435/K*r^-0.921, T/0.878*r^0.749,   0.482*T*r^1.137];    % perturbación (ideal)
P.ITAE_ref = [0.965/K*r^-0.855, T/(0.796-0.147*r), 0.308*T*r^0.929];
P.ITAE_pert= [1.357/K*r^-0.947, T/0.842*r^0.738,   0.381*T*r^0.995];

pidReal  = @(p) p(1)*(1 + tf(1,[p(2) 0]))*tf([p(3) 1],1);   % serie, sin filtro
pidIdeal = @(p) p(1)*(1 + tf(1,[p(2) 0]) + tf([p(3) 0],1));

%% Pasos 5 a 11: evaluar cada controlador
nombres = fieldnames(P);
tsim = 0:0.01:40;
figure('Name','Referencia');   hold on; grid on; title('Escalón en la referencia');
figure('Name','Perturbación'); hold on; grid on; title('Escalón en la perturbación F');
leg = {};
for i = 1:numel(nombres)
    p = P.(nombres{i});
    fprintf('%-10s Kc = %.3f  Ti = %.3f  Td = %.3f\n', nombres{i}, p);
    formas = {'ideal'};
    if i <= 3, formas = {'real', 'ideal'}; end   % ZN y CC: ambas formas
    for j = 1:numel(formas)
        if strcmp(formas{j}, 'real'), Gc = pidReal(p); else, Gc = pidIdeal(p); end
        Tr = feedback(Gc*G1, 1);        % C/R
        Td_ = G2*feedback(1, Gc*G1);    % C/F
        figure(2); step(Tr, tsim);
        figure(3); step(Td_, tsim);
        leg{end+1} = sprintf('%s %s', strrep(nombres{i},'_','-'), formas{j}); %#ok<SAGROW>
        Sr = stepinfo(Tr);
        fprintf('   %-5s ref: Mp = %5.1f%%  ts = %5.2f min\n', formas{j}, Sr.Overshoot, Sr.SettlingTime);
    end
end
figure(2); legend(leg); figure(3); legend(leg);

%% Paso 12: ZN (Ku) serie -> ideal -> paralelo, para Simulink
p = P.ZN_Ku;                       % [K'c T'i T'd]
Kc = p(1)*(1 + p(3)/p(2));
Ti = p(2) + p(3);
Td = p(2)*p(3)/(p(2) + p(3));
Pp = Kc;  Ip = Kc/Ti;  Dp = Kc*Td;
Kb = 1/sqrt(Ti*Td);                % ganancia anti-windup (back-calculation)
N  = 20;                           % coeficiente del filtro derivativo [rad/min]
fprintf('\nIdeal: Kc = %.4f  Ti = %.4f  Td = %.4f\n', Kc, Ti, Td);
fprintf('Paralelo: P = %.4f  I = %.4f  D = %.4f  Kb = %.4f  N = %g\n', Pp, Ip, Dp, Kb, N);
