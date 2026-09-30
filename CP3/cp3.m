%% CP-3: Compensación serie (módulo óptimo y módulo simétrico)
clear; clc; close all;
MO = @(Gp, Tu) minreal(tf(1, 2*Tu*[Tu 1 0]) / Gp);               % Kr = 1
MS = @(Gp, Tu) minreal(tf([4*Tu 1], 8*Tu^2*[Tu 1 0 0]) / Gp);

%% Ejercicio 1 (simbólico, requiere Symbolic Math Toolbox)
syms s T1 T2 T3 K1 K2 K3 positive
plantas = {K1*K2/((T1*s+1)*s), K1*K2/((T1*s+1)*(T2*s+1)), ...
           K1*K2*K3/((T1*s+1)*(T2*s+1)*s), K1*K2*K3/((T1*s+1)*(T2*s+1)*(T3*s+1))};
for i = 1:4
    Gp = plantas{i};
    Gmo = simplify(1/(2*T1*s*(T1*s+1)*Gp));
    Gms = simplify((4*T1*s+1)/(8*T1^2*s^2*(T1*s+1)*Gp));
    fprintf('\nPlanta %d\n  MO: %s\n      = %s\n', i, char(factor(Gmo)), char(partfrac(Gmo, s)));
    fprintf('  MS: %s\n      = %s\n', char(factor(Gms)), char(partfrac(Gms, s)));
end

%% Ejercicio 2
t = 0:0.001:40;
casos = {tf(100, [1 10 0]), 0.1, 4; tf(2, conv([1 0], conv([0.8 1], [1 0.5]))), 0.8, 40};
for i = 1:2
    Gp = casos{i,1}; Tu = casos{i,2}; tf_ = casos{i,3};
    Gco = MO(Gp, Tu), Gcs = MS(Gp, Tu)
    tt = t(t <= tf_);
    figure('Name', sprintf('Ej2 - Gp%d', i));
    subplot(3,1,1), step(feedback(Gco*Gp,1), feedback(Gcs*Gp,1), tt)
    title('Escalón en la referencia'), legend('MO','MS'), grid on
    subplot(3,1,2), lsim(feedback(Gco*Gp,1), feedback(Gcs*Gp,1), tt, tt)
    title('Rampa en la referencia'), legend('MO','MS'), grid on
    subplot(3,1,3), step(feedback(Gp,Gco), feedback(Gp,Gcs), tt)
    title('Escalón en la perturbación (entrada de la planta)'), legend('MO','MS'), grid on
    % Errores en estado estable (verificación)
    for G = {Gco, Gcs}
        L = G{1}*Gp;
        fprintf('  e_escalón = %.4f  e_rampa = %.4f  y_pert = %.4f\n', ...
            dcgain(1/(1+L)), dcgain(minreal(tf(1,[1 0])/(1+L))), dcgain(feedback(Gp, G{1})));
    end
end

%% Ejercicio 3: U -> 1/(0.01s+1) -> 1/(0.1s+1) -> V -> 1/(0.1s+1) -> Y
Ga = tf(1,[0.01 1]); Gb = tf(1,[0.1 1]); Gc_ = tf(1,[0.1 1]);
G3 = Ga*Gb*Gc_;
% a) Un solo controlador por MO (Tu = 0.01): PID
Gpid = MO(G3, 0.01)                         % = 10 + 50/s + 0.5 s
% b) Cascada: interno MO sobre Ga*Gb (Tu = 0.01), externo MO sobre 1/(0.02s+1)*Gc (Tu = 0.02)
Gc1 = MO(Ga*Gb, 0.01)                       % = 5 + 50/s
Gc2 = MO(tf(1,[0.02 1])*Gc_, 0.02)          % = 2.5 + 25/s
Gin = feedback(Gc1*Ga*Gb, 1);
figure('Name','Ej3 lineal');
step(feedback(Gpid*G3, 1), feedback(Gc2*Gin*Gc_, 1), 0.5); grid on; legend('PID único','Cascada')
% Perturbación en el mando: entra en la entrada de Ga
% Y/D con un solo PID:  feedback(G3, Gpid)
% Cascada: v = Ga*Gb*(d - Gc1*v - Gc1*Gc2*Gc_*v)  ->  Y/D = Gc_*Ga*Gb/(1 + Ga*Gb*Gc1*(1 + Gc2*Gc_))
figure('Name','Ej3 perturbación');
step(feedback(G3, Gpid), Gc_*feedback(Ga*Gb, Gc1*(1 + Gc2*Gc_)), 1); grid on
legend('PID único','Cascada');
% Saturación ±10 y antiwindup: usar el modelo Simulink (ver README).

%% Ejercicio 4
Gi  = tf(1,[0.1 1])*tf(1,[0.01 1]);
Gc1 = MO(Gi, 0.01)                                    % = 5 + 50/s  (PI)
Go  = tf(1,[0.02 1])*tf(1,[2 1])*tf(1,[1 0]);         % lazo interno aproximado * planta externa
Gc2mo = MO(Go, 0.02)                                  % = 25 + 50 s (PD)
Gc2ms = MS(Go, 0.02)                                  % = 650 + 312.5/s + 50 s (PID)
Ginner = feedback(Gc1*Gi, 1);
Gext = Ginner*tf(1,[2 1])*tf(1,[1 0]);
figure('Name','Ej4 lineal');
step(feedback(Gc2mo*Gext, 1), feedback(Gc2ms*Gext, 1), 3); grid on; legend('Gc2 MO','Gc2 MS')
