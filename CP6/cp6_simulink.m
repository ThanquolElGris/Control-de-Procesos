%% CP-6: construye, simula y dibuja los modelos Simulink de los 5 ejercicios
% Genera cp6_ej1_zn.slx, cp6_ej1_smith.slx, cp6_ej2_zn.slx, cp6_ej2_comp.slx, cp6_ej3.slx,
% cp6_ej4.slx y cp6_ej5.slx, una imagen de cada diagrama (diag_*.png), los gráficos
% ej3_simulink.png y ej5_simulink.png, y verifica que los resultados coincidan con cp6.m.
% Disposición: perturbación arriba, lazo principal al centro, modelos/compensadores y realimentación abajo.
clear; clc; close all;
set(groot, 'defaultFigureVisible', 'off');
warning('off', 'all');

%% ======================== Ejercicio 1: calentador ========================
Ku = 0.5289;  Tu = 2.5665;                       % de cp6.m (margin)
Kc = Ku/2.2;  Ti = Tu/1.2;
% ---- a) PI por ZN con saturación 0-10 V ----
m = 'cp6_ej1_zn';  nuevo(m);
blk(m, 'Ref',   'simulink/Sources/Step', {'Time','1','Before','0','After','2'}, [0 0]);
blk(m, 'Sum_e', 'simulink/Math Operations/Sum', {'Inputs','|+-'}, [1 0]);
blk(m, 'PI_ZN', 'simulink/Continuous/PID Controller', {'Controller','PI','Form','Parallel', ...
    'P',sprintf('%.4f',Kc),'I',sprintf('%.4f',Kc/Ti)}, [2 0]);
blk(m, 'Sat',   'simulink/Discontinuities/Saturation', {'UpperLimit','10','LowerLimit','0'}, [3 0]);
blk(m, 'Pert',  'simulink/Sources/Step', {'Time','10','Before','0','After','0.1'}, [3.5 -1.3]);
blk(m, 'Sum_d', 'simulink/Math Operations/Sum', {'Inputs','++|'}, [4 0]);
blk(m, 'G',     'simulink/Continuous/Transfer Fcn', {'Numerator','5','Denominator','[1 1]'}, [5 0]);
blk(m, 'L',     'simulink/Continuous/Transport Delay', {'DelayTime','0.8'}, [6 0]);
sal(m, 'T', [7 0]);  sal(m, 'u', [3.5 1]);  sal(m, 'ref', [0.6 1]);
lin(m,'Ref/1','Sum_e/1','r');  lin(m,'Sum_e/1','PI_ZN/1','e');  lin(m,'PI_ZN/1','Sat/1');
lin(m,'Sat/1','Sum_d/2','u');  lin(m,'Pert/1','Sum_d/1','d');  lin(m,'Sum_d/1','G/1');
lin(m,'G/1','L/1');            lin(m,'L/1','T/1','T');         lin(m,'L/1','Sum_e/2');
lin(m,'Sat/1','u/1');          lin(m,'Ref/1','ref/1');
alcance(m, 'Ref/1', 'L/1', [6.5 -1.3]);
terminar(m, '20', '0.002');
o = sim(m);  [t,y] = senal(o,1);  [~,u] = senal(o,2);
fprintf('Ej1 ZN:     Mp = %.1f %%  u max = %.3f\n', (max(y(t<10))-2)/2*100, max(u));
close_system(m, 0);

% ---- b) y c) PI por cancelación + predictor de Smith (filtrado o no) ----
m = 'cp6_ej1_smith';  nuevo(m);
blk(m, 'Ref',   'simulink/Sources/Step', {'Time','1','Before','0','After','2'}, [0 0]);
blk(m, 'Sum_e', 'simulink/Math Operations/Sum', {'Inputs','+--','IconShape','rectangular'}, [1 0.25]);
blk(m, 'PI_canc','simulink/Continuous/PID Controller', {'Controller','PI','Form','Parallel','P','5','I','5'}, [2 0]);
blk(m, 'Sat',   'simulink/Discontinuities/Saturation', {'UpperLimit','10','LowerLimit','0'}, [3 0]);
blk(m, 'Pert',  'simulink/Sources/Step', {'Time','10','Before','0','After','0.1'}, [3.5 -1.3]);
blk(m, 'Sum_d', 'simulink/Math Operations/Sum', {'Inputs','++|'}, [4 0]);
blk(m, 'G',     'simulink/Continuous/Transfer Fcn', {'Numerator','5','Denominator','[1 1]'}, [5 0]);
blk(m, 'L',     'simulink/Continuous/Transport Delay', {'DelayTime','0.8'}, [6 0]);
% Modelo del predictor (nominal): P = 5/(s+1) sin y con retardo
blk(m, 'P_mod', 'simulink/Continuous/Transfer Fcn', {'Numerator','5','Denominator','[1 1]'}, [4 1.3]);
blk(m, 'P_mod2','simulink/Continuous/Transfer Fcn', {'Numerator','5','Denominator','[1 1]'}, [4 2.4]);
blk(m, 'L_mod', 'simulink/Continuous/Transport Delay', {'DelayTime','0.8'}, [5 2.4]);
blk(m, 'Sum_p', 'simulink/Math Operations/Sum', {'Inputs','+-|'}, [6 2.4]);
blk(m, 'Fr',    'simulink/Continuous/Transfer Fcn', {'Numerator','[0.7412 1]','Denominator','[0.4 1]'}, [7 2.4]);
sal(m, 'T', [7.3 0]);  sal(m, 'u', [3.5 -0.8]);  sal(m, 'ref', [0.6 -0.8]);
lin(m,'Ref/1','Sum_e/1','r');  lin(m,'Sum_e/1','PI_canc/1','e');  lin(m,'PI_canc/1','Sat/1');
lin(m,'Sat/1','Sum_d/2','u');  lin(m,'Pert/1','Sum_d/1','d');    lin(m,'Sum_d/1','G/1');
lin(m,'G/1','L/1');            lin(m,'L/1','T/1','T');
lin(m,'Sat/1','P_mod/1');      lin(m,'P_mod/1','Sum_e/2','y_m (sin retardo)');
lin(m,'Sat/1','P_mod2/1');     lin(m,'P_mod2/1','L_mod/1');      lin(m,'L_mod/1','Sum_p/2','y_md');
lin(m,'L/1','Sum_p/1');        lin(m,'Sum_p/1','Fr/1','e_pred'); lin(m,'Fr/1','Sum_e/3');
lin(m,'Sat/1','u/1');          lin(m,'Ref/1','ref/1');
alcance(m, 'Ref/1', 'L/1', [6.5 -1.3]);
terminar(m, '20', '0.002');
for caso = {'clasico','filtrado'}
    if strcmp(caso{1}, 'clasico'), set_param([m '/Fr'], 'Numerator','1', 'Denominator','1'); end
    o = sim(m);  [t,y] = senal(o,1);  j = t >= 10;  dev = y(j) - 2;
    b = find(abs(dev) > 0.02*max(abs(dev)), 1, 'last');
    fprintf('Ej1 Smith %-9s: ts(ref) = %.2f  pico ante d = %.4f  recuperación = %.2f min\n', caso{1}, ...
            t(find(abs(y(t<10)-2) > 0.04, 1, 'last'))-1, max(dev), t(find(j,1)+b-1)-10);
    set_param([m '/Fr'], 'Numerator','[0.7412 1]', 'Denominator','[0.4 1]');
end
set_param([m '/Fr'], 'Numerator','1', 'Denominator','1');  set_param([m '/L'], 'DelayTime','0.88');
o = sim(m);  [t,y] = senal(o,1);
fprintf('Ej1 Smith con L real +10 %%: max T = %.2f, T entre 15 y 20 min oscila entre %.2f y %.2f\n', max(y), min(y(t>15)), max(y(t>15)));
set_param([m '/L'], 'DelayTime','0.8');  set_param([m '/Fr'], 'Numerator','[0.7412 1]', 'Denominator','[0.4 1]');
save_system(m);  close_system(m, 0);

%% ======================== Ejercicio 2: respuesta inversa con válvula ========================
for caso = {'zn','comp'}
    m = ['cp6_ej2_' caso{1}];  nuevo(m);
    blk(m, 'Ref', 'simulink/Sources/Step', {'Time','1','Before','0','After','1'}, [0 0]);
    if strcmp(caso{1}, 'zn')
        blk(m, 'Sum_e', 'simulink/Math Operations/Sum', {'Inputs','|+-'}, [1 0]);
        blk(m, 'PI_ZN', 'simulink/Continuous/PID Controller', {'Controller','PI','Form','Parallel', ...
            'P',sprintf('%.4f',1.6154/2.2),'I',sprintf('%.4f',(1.6154/2.2)/(6.8305/1.2))}, [2 0]);
        ctrl = 'PI_ZN';
    else
        blk(m, 'Sum_e', 'simulink/Math Operations/Sum', {'Inputs','+--','IconShape','rectangular'}, [1 0.25]);
        blk(m, 'PID_MO', 'simulink/Continuous/PID Controller', {'Form','Parallel','P','2','I','0.5','D','1.5','N','20'}, [2 0]);
        blk(m, 'Gcomp', 'simulink/Continuous/Transfer Fcn', {'Numerator','[2 0]', ...
            'Denominator','[1.5 5 4.5 1]'}, [4 1.6]);
        ctrl = 'PID_MO';
    end
    blk(m, 'Pert',  'simulink/Sources/Step', {'Time','30','Before','0','After','1'}, [2.5 -1.3]);
    blk(m, 'Sum_a', 'simulink/Math Operations/Sum', {'Inputs','++|'}, [3 0]);
    blk(m, 'Gv',    'simulink/Continuous/Transfer Fcn', {'Numerator','2','Denominator','[0.5 1]'}, [4 0]);
    blk(m, 'G1',    'simulink/Continuous/Transfer Fcn', {'Numerator','2','Denominator','[3 1]'}, [5 -0.7]);
    blk(m, 'G2',    'simulink/Continuous/Transfer Fcn', {'Numerator','1','Denominator','[1 1]'}, [5 0.7]);
    blk(m, 'Sum_h', 'simulink/Math Operations/Sum', {'Inputs','+|-'}, [6 0]);
    sal(m, 'h', [7 0]);  sal(m, 'u', [2.6 1]);  sal(m, 'ref', [0.6 1]);
    lin(m,'Ref/1','Sum_e/1','r');  lin(m,'Sum_e/1',[ctrl '/1'],'e');  lin(m,[ctrl '/1'],'Sum_a/2','u');
    lin(m,'Pert/1','Sum_a/1','dist');  lin(m,'Sum_a/1','Gv/1');  lin(m,'Gv/1','G1/1');  lin(m,'Gv/1','G2/1');
    lin(m,'G1/1','Sum_h/1');  lin(m,'G2/1','Sum_h/2');  lin(m,'Sum_h/1','h/1','h');  lin(m,'Sum_h/1','Sum_e/2');
    lin(m,[ctrl '/1'],'u/1');  lin(m,'Ref/1','ref/1');
    if strcmp(caso{1}, 'comp'), lin(m,[ctrl '/1'],'Gcomp/1');  lin(m,'Gcomp/1','Sum_e/3','y_c'); end
    alcance(m, 'Ref/1', 'Sum_h/1', [6.5 -1.3]);
    terminar(m, '60', '0.005');
    o = sim(m);  [t,y] = senal(o,1);
    fprintf('Ej2 %-4s: Mp = %.1f %%  mínimo = %.3f  pico ante d = %+.3f\n', caso{1}, (max(y(t<30))-1)*100, min(y), max(y(t>=30))-1);
    close_system(m, 0);
end

%% ======================== Ejercicio 3 ========================
a = 2.5;
m = 'cp6_ej3';  nuevo(m);
blk(m, 'Ref',   'simulink/Sources/Step', {'Time','1','Before','0','After','1'}, [0 0]);
blk(m, 'Sum_e', 'simulink/Math Operations/Sum', {'Inputs','+--','IconShape','rectangular'}, [1 0.25]);
blk(m, 'Gc',    'simulink/Continuous/Transfer Fcn', {'Numerator','[4 1]','Denominator',sprintf('[%g 0]',a)}, [2 0]);
blk(m, 'Gv',    'simulink/Continuous/Transfer Fcn', {'Numerator','2','Denominator','[2 2]'}, [3.2 0]);
blk(m, 'Gp',    'simulink/Continuous/Transfer Fcn', {'Numerator','[-5 1]','Denominator','[4 1]'}, [4.4 0]);
blk(m, 'Gcomp', 'simulink/Continuous/Transfer Fcn', {'Numerator','[5 0]','Denominator','[4 5 1]'}, [3.8 1.4]);
sal(m, 'med', [5.5 0]);  sal(m, 'man', [2.8 -0.9]);  sal(m, 'ref', [0.6 -0.9]);
lin(m,'Ref/1','Sum_e/1','r');  lin(m,'Sum_e/1','Gc/1','e');  lin(m,'Gc/1','Gv/1','man');  lin(m,'Gv/1','Gp/1');
lin(m,'Gp/1','med/1','med');   lin(m,'Gp/1','Sum_e/2');      lin(m,'Gc/1','Gcomp/1');      lin(m,'Gcomp/1','Sum_e/3','y_c');
lin(m,'Gc/1','man/1');         lin(m,'Ref/1','ref/1');
alcance(m, 'Ref/1', 'Gp/1', [5.3 -1.3]);
terminar(m, '30', '0.01');
o = sim(m);  [t,med] = senal(o,1);  [~,man] = senal(o,2);
f = figure('Position',[100 100 800 550]);
subplot(2,1,1); plot(t, med, t, double(t >= 1), 'k--'); grid on; ylabel('med');
title(sprintf('Simulink cp6\\_ej3: compensador de RI + PI (a = %g)', a));
subplot(2,1,2); plot(t, man); grid on; ylabel('man'); xlabel('t');
guardar(f, 'ej3_simulink.png');
fprintf('Ej3: Mp = %.2f %%, mínimo = %.3f, med(30) = %.4f\n', (max(med)-1)*100, min(med), med(end));
close_system(m, 0);

%% ======================== Ejercicio 4 ========================
m = 'cp6_ej4';  nuevo(m);
set_param(m, 'InitFcn', 'k1 = 4; k2 = 2; T = 3;   % caso 1 (cambiar a k1 = 2; k2 = 4 para el caso 2)');
blk(m, 'Ref',   'simulink/Sources/Step', {'Time','1','Before','0','After','1'}, [0 0]);
blk(m, 'Sum_e', 'simulink/Math Operations/Sum', {'Inputs','+--','IconShape','rectangular'}, [1 0.25]);
blk(m, 'Gc_MO', 'simulink/Math Operations/Gain', {'Gain','-1/(2*T*k2)'}, [2 0]);
blk(m, 'G_a',   'simulink/Continuous/Transfer Fcn', {'Numerator','[k1]','Denominator','[T 1]'}, [3.5 -0.7]);
blk(m, 'G_b',   'simulink/Continuous/Transfer Fcn', {'Numerator','[k2]','Denominator','[T 1 0]'}, [3.5 0.7]);
blk(m, 'Sum_y', 'simulink/Math Operations/Sum', {'Inputs','+|-'}, [4.6 0]);
blk(m, 'Gcomp', 'simulink/Continuous/Transfer Fcn', {'Numerator','-k1','Denominator','[T 1]'}, [3.5 1.8]);
sal(m, 'y', [5.6 0]);  sal(m, 'u', [2.6 -1]);  sal(m, 'ref', [0.6 -1]);
lin(m,'Ref/1','Sum_e/1','r');  lin(m,'Sum_e/1','Gc_MO/1','e');  lin(m,'Gc_MO/1','G_a/1','u');  lin(m,'Gc_MO/1','G_b/1');
lin(m,'G_a/1','Sum_y/1');      lin(m,'G_b/1','Sum_y/2');         lin(m,'Sum_y/1','y/1','y');    lin(m,'Sum_y/1','Sum_e/2');
lin(m,'Gc_MO/1','Gcomp/1');    lin(m,'Gcomp/1','Sum_e/3','y_c'); lin(m,'Gc_MO/1','u/1');        lin(m,'Ref/1','ref/1');
alcance(m, 'Ref/1', 'Sum_y/1', [5.4 -1.4]);
terminar(m, '40', '0.01');
for caso = {'k1 = 4; k2 = 2; T = 3;', 'k1 = 2; k2 = 4; T = 3;'}
    set_param(m, 'InitFcn', caso{1});  o = sim(m);  [~,y] = senal(o,1);
    fprintf('Ej4 %s  Mp = %.2f %%  mínimo = %.4f\n', caso{1}, (max(y)-1)*100, min(y));
end
set_param(m, 'InitFcn', 'k1 = 4; k2 = 2; T = 3;   % caso 1 (cambiar a k1 = 2; k2 = 4 para el caso 2)');
save_system(m);  close_system(m, 0);

%% ======================== Ejercicio 5 ========================
Kp1 = 1.2; tau = 30; L = 10; q = L/tau;                         % PID ideal por ITAE (referencia)
Kc = 0.965/Kp1*q^-0.855;  Ti = tau/(0.796 - 0.147*q);  Td = 0.308*tau*q^0.929;  N = 10;
m = 'cp6_ej5';  nuevo(m);
blk(m, 'Ref',  'simulink/Sources/Step', {'Time','10','Before','0','After','1'}, [0 0]);
blk(m, 'Sum_e','simulink/Math Operations/Sum', {'Inputs','|+-'}, [1 0]);
blk(m, 'PID',  'simulink/Continuous/PID Controller', {'Form','Ideal','P',sprintf('%.4f',Kc),'I',sprintf('%.5f',1/Ti), ...
     'D',sprintf('%.4f',Td),'N',sprintf('%.4f',N/Td)}, [2 0]);
% Planta (columna)
blk(m, 'XF',   'simulink/Sources/Step', {'Time','400','Before','0','After','1'}, [2.6 -2.2]);
blk(m, 'Gp1',  'simulink/Continuous/Transfer Fcn', {'Numerator','1.2','Denominator','[30 1]'}, [3.5 -0.6]);
blk(m, 'Lp1',  'simulink/Continuous/Transport Delay', {'DelayTime','10'}, [4.4 -0.6]);
blk(m, 'Gd1',  'simulink/Continuous/Transfer Fcn', {'Numerator','0.9','Denominator','[70 1]'}, [3.5 -1.6]);
blk(m, 'Ld1',  'simulink/Continuous/Transport Delay', {'DelayTime','5'}, [4.4 -1.6]);
blk(m, 'Sum_XD','simulink/Math Operations/Sum', {'Inputs','++|'}, [5.2 -0.6]);
blk(m, 'Gp2',  'simulink/Continuous/Transfer Fcn', {'Numerator','1','Denominator','[20 1]'}, [3.5 0.6]);
blk(m, 'Gd2',  'simulink/Continuous/Transfer Fcn', {'Numerator','0.2','Denominator','[60 1]'}, [3.5 1.6]);
blk(m, 'Ld2',  'simulink/Continuous/Transport Delay', {'DelayTime','5'}, [4.4 1.6]);
blk(m, 'Sum_T','simulink/Math Operations/Sum', {'Inputs','+|+'}, [5.2 0.6]);
% Estimador: XDest = Gp1 R - (Gd1/Gd2) Gp2 R + (Gd1/Gd2) T
blk(m, 'Mod_Gp1',  'simulink/Continuous/Transfer Fcn', {'Numerator','1.2','Denominator','[30 1]'}, [3.5 2.8]);
blk(m, 'Mod_Lp1',  'simulink/Continuous/Transport Delay', {'DelayTime','10'}, [4.4 2.8]);
blk(m, 'Mod_rGp2', 'simulink/Continuous/Transfer Fcn', {'Numerator','[270 4.5]','Denominator','[1400 90 1]'}, [3.5 3.8]);
blk(m, 'Mod_r',    'simulink/Continuous/Transfer Fcn', {'Numerator','[270 4.5]','Denominator','[70 1]'}, [6.3 1.8]);
blk(m, 'Sum_est',  'simulink/Math Operations/Sum', {'Inputs','+-+','IconShape','rectangular'}, [7.3 2.8]);
sal(m, 'XD', [6.3 -0.6]);  sal(m, 'XDest', [8.2 2.8]);  sal(m, 'R', [2.7 -1]);
lin(m,'Ref/1','Sum_e/1','r');  lin(m,'Sum_e/1','PID/1','e');
lin(m,'PID/1','Gp1/1','R');    lin(m,'Gp1/1','Lp1/1');   lin(m,'Lp1/1','Sum_XD/2');
lin(m,'XF/1','Gd1/1','X_F');   lin(m,'Gd1/1','Ld1/1');   lin(m,'Ld1/1','Sum_XD/1');  lin(m,'Sum_XD/1','XD/1','X_D (no medida)');
lin(m,'PID/1','Gp2/1');        lin(m,'Gp2/1','Sum_T/1'); lin(m,'XF/1','Gd2/1');      lin(m,'Gd2/1','Ld2/1');  lin(m,'Ld2/1','Sum_T/2');
lin(m,'PID/1','Mod_Gp1/1');    lin(m,'Mod_Gp1/1','Mod_Lp1/1');  lin(m,'Mod_Lp1/1','Sum_est/1');
lin(m,'PID/1','Mod_rGp2/1');   lin(m,'Mod_rGp2/1','Sum_est/2'); lin(m,'Sum_T/1','Mod_r/1','T (medida)');  lin(m,'Mod_r/1','Sum_est/3');
lin(m,'Sum_est/1','XDest/1','X_D inferida');  lin(m,'Sum_est/1','Sum_e/2');  lin(m,'PID/1','R/1');
terminar(m, '800', '0.1');
o = sim(m);  [t,xd] = senal(o,1);  [~,xde] = senal(o,2);  [~,R] = senal(o,3);
f = figure('Position',[100 100 800 550]);
subplot(2,1,1); plot(t, xd, t, xde, '--', t, double(t >= 10), 'k:'); grid on; ylabel('X_D');
legend('X_D real','X_D inferida','referencia','Location','southeast');
title('Simulink cp6\_ej5: control inferencial (r = 1 en t = 10, X_F = 1 en t = 400)');
subplot(2,1,2); plot(t, R); grid on; ylabel('reflujo R'); xlabel('t [min]');
guardar(f, 'ej5_simulink.png');
fprintf('Ej5: Mp = %.2f %%, XD(399) = %.4f, XD(800) = %.4f\n', (max(xd(t<400))-1)*100, interp1(t,xd,399), xd(end));
close_system(m, 0);

%% ======================== funciones ========================
function nuevo(m)
    if bdIsLoaded(m), close_system(m, 0); end
    if exist([m '.slx'], 'file'), delete([m '.slx']); end
    new_system(m);  set_param(m, 'SaveOutput','on', 'SaveFormat','Dataset', 'OutputSaveName','yout');
end
function p = pos(cr, w, h)
    % columna/fila de la grilla -> rectángulo [izq arriba der abajo]
    x = 60 + cr(1)*170;  y = 260 + cr(2)*80;
    p = round([x-w/2 y-h/2 x+w/2 y+h/2]);
end
function blk(m, nombre, lib, par, cr)
    if contains(lib, {'Transfer Fcn','PID'}),   w = 120; h = 46;
    elseif contains(lib, 'Sum')
        if any(strcmp(par, 'rectangular')), w = 24; h = 60; else, w = 30; h = 30; par = [par {'IconShape','round'}]; end
    elseif contains(lib, 'Transport'),           w = 50;  h = 40;
    else,                                        w = 40;  h = 36;
    end
    add_block(lib, [m '/' nombre], 'Position', pos(cr, w, h), par{:});
end
function sal(m, nombre, cr)
    add_block('simulink/Sinks/Out1', [m '/' nombre], 'Position', pos(cr, 30, 16));
end
function lin(m, a, b, etiqueta)
    h = add_line(m, a, b, 'autorouting', 'smart');
    if nargin > 3, set_param(h, 'Name', etiqueta); end
end
function alcance(m, ref, y, cr)
    add_block('simulink/Signal Routing/Mux', [m '/Mux'], 'Inputs', '2', 'Position', pos(cr, 6, 40));
    add_block('simulink/Sinks/Scope', [m '/Scope'], 'Position', pos(cr + [0.5 0], 34, 34));
    add_line(m, ref, 'Mux/1', 'autorouting','smart');  add_line(m, y, 'Mux/2', 'autorouting','smart');
    add_line(m, 'Mux/1', 'Scope/1', 'autorouting','smart');
end
function terminar(m, tf, maxstep)
    set_param(m, 'StopTime', tf, 'MaxStep', maxstep);
    save_system(m);
    print(['-s' m], '-dpng', '-r110', ['diag_' m '.png']);
end
function [t, y] = senal(o, k)
    t = o.yout{k}.Values.Time;  y = o.yout{k}.Values.Data;
end
function guardar(f, nombre)
    try, theme(f, 'light'); end
    exportgraphics(f, nombre, 'Resolution', 110);
end
