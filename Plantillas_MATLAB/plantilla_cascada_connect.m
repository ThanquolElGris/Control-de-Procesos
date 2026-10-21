%% PLANTILLA: cascada (o cualquier diagrama) con connect()
% connect() arma el sistema a partir de NOMBRES de señales, así no hay que deducir
% a mano las FT de lazo cerrado. Sirve para lazo simple, cascada, perturbación en
% cualquier punto, feedforward, etc.
% Ejemplo: CP-4 ej. 1  (man -> Gv -> +dist -> G1 -> med1 -> G2 -> e^-s -> med2)
clear; clc; close all;
s = tf('s');

% ---- 1) Bloques de la planta, con nombres de entrada y salida ----
Gv = tf(2, [1 2]);         Gv.InputName = 'u';   Gv.OutputName = 'uv';
G1 = tf(5, [2 1]);         G1.InputName = 'a';   G1.OutputName = 'med1';
G2 = tf(3, [5 1], 'InputDelay', 1);  G2.InputName = 'med1';  G2.OutputName = 'med2';
S1 = sumblk('a = uv + d');                    % punto donde entra la perturbación

% ---- 2) Diseño ----
Gi  = tf(2,[1 2])*tf(5,[2 1]);                % planta interna (sin nombres)
Gc1 = sintesis('MO', Gi, 0.5);  tf2pid(Gc1);  % interno MO
Gcl1 = feedback(Gc1*Gi, 1);
Gext = Gcl1*tf(3,[5 1],'InputDelay',1);       % lo que "ve" el externo
[Ku, Tu] = ku_tu(Gext);
c = sintonia('ZN-LC', 'PID', Ku, Tu);
N = 100;
C2 = tf(pid(c.P, c.I, c.D, 1/N));             % PID con filtro (propio)

% ---- 3) Controladores y sumadores del lazo ----
C2.InputName = 'e2';  C2.OutputName = 'r1';   % externo: e2 -> referencia del interno
C1 = tf(Gc1);  C1.InputName = 'e1';  C1.OutputName = 'u';
S2 = sumblk('e2 = r - med2');
S3 = sumblk('e1 = r1 - med1');

% ---- 4) Sistema completo: entradas {r, d}, salidas {med2, u, e2} ----
T = connect(Gv, G1, G2, S1, C1, C2, S2, S3, {'r','d'}, {'med2','u','e2'});

% ---- 5) Simulación: escalón en r en t = 1 y en d en t = 60 ----
t = (0:0.01:120)';
entradas = [double(t >= 1), double(t >= 60)];
Y = lsim(T, entradas, t);
figure
subplot(3,1,1), plot(t, entradas(:,1), 'k--', t, Y(:,1)), grid on, legend('r','med2'), title('Referencia y salida')
subplot(3,1,2), plot(t, Y(:,2)), grid on, title('Acción de control u')
subplot(3,1,3), plot(t, Y(:,3)), grid on, title('Error'), xlabel('t [s]')

% Métricas por separado
Tr = T('med2','r');  Td = T('med2','d');
stepinfo(Tr)
fprintf('y_ss ante perturbación unitaria = %.4g\n', dcgain(Td));
% Para lazo simple: quitar C1, S3 y usar C2.OutputName = 'u'.
