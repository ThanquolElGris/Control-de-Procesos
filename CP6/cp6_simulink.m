%% CP-6: construye y simula los modelos Simulink de los ejercicios 3 y 5
% Genera cp6_ej3.slx y cp6_ej5.slx (se pueden abrir y modificar a mano) y los gráficos
% ej3_simulink.png y ej5_simulink.png.
clear; clc; close all;
set(groot, 'defaultFigureVisible', 'off');

%% ======================== Ejercicio 3 ========================
% r -> (+ - -) -> Gc -> u -> Gv -> Gp -> med ;  u -> Gcomp -> yc (se suma a la realimentación)
a = 2.5;                                          % ver cp6.m: menor a con Mp(med) <= 4 %
m = 'cp6_ej3';  nuevo(m);
blk(m, 'Ref',   'simulink/Sources/Step',            [30 100], {'Time','1','Before','0','After','1'});
blk(m, 'Suma',  'simulink/Math Operations/Sum',     [110 100], {'Inputs','+--','IconShape','round'});
blk(m, 'Gc',    'simulink/Continuous/Transfer Fcn', [180 100], {'Numerator','[4 1]','Denominator',sprintf('[%g 0]',a)});
blk(m, 'Gv',    'simulink/Continuous/Transfer Fcn', [300 100], {'Numerator','[2]','Denominator','[2 2]'});
blk(m, 'Gp',    'simulink/Continuous/Transfer Fcn', [420 100], {'Numerator','[-5 1]','Denominator','[4 1]'});
blk(m, 'Gcomp', 'simulink/Continuous/Transfer Fcn', [300 220], {'Numerator','[5 0]','Denominator','[4 5 1]'});
blk(m, 'med',   'simulink/Sinks/Out1',              [540 100], {});
blk(m, 'man',   'simulink/Sinks/Out1',              [300 300], {});
lin(m, 'Ref/1','Suma/1');  lin(m, 'Suma/1','Gc/1');  lin(m, 'Gc/1','Gv/1');  lin(m, 'Gv/1','Gp/1');
lin(m, 'Gp/1','med/1');    lin(m, 'Gp/1','Suma/2');  lin(m, 'Gc/1','Gcomp/1'); lin(m, 'Gcomp/1','Suma/3');
lin(m, 'Gc/1','man/1');
set_param(m, 'StopTime','30', 'MaxStep','0.01');  save_system(m);
o = sim(m);  y = o.yout;
t = y{1}.Values.Time;  med = y{1}.Values.Data;  man = y{2}.Values.Data;
f = figure('Position',[100 100 800 550]);
subplot(2,1,1); plot(t, med, t, double(t >= 1), 'k--'); grid on; ylabel('med');
title(sprintf('Simulink cp6\\_ej3: compensador de RI + PI (a = %g)', a));
subplot(2,1,2); plot(t, man); grid on; ylabel('man'); xlabel('t');
guardar(f, 'ej3_simulink.png');
fprintf('Ej3 Simulink: Mp = %.2f %%, mínimo = %.3f, med(30) = %.4f\n', (max(med)-1)*100, min(med), med(end));
close_system(m, 0);

%% ======================== Ejercicio 5 ========================
Kp1 = 1.2; tau = 30; L = 10; q = L/tau;                         % PID ideal por ITAE (referencia)
Kc = 0.965/Kp1*q^-0.855;  Ti = tau/(0.796 - 0.147*q);  Td = 0.308*tau*q^0.929;  N = 10;
m = 'cp6_ej5';  nuevo(m);
blk(m, 'Ref',  'simulink/Sources/Step', [30 60],  {'Time','10','Before','0','After','1'});
blk(m, 'XF',   'simulink/Sources/Step', [30 330], {'Time','400','Before','0','After','1'});
blk(m, 'Suma', 'simulink/Math Operations/Sum', [110 60], {'Inputs','+-','IconShape','round'});
blk(m, 'PID',  'simulink/Continuous/PID Controller', [180 60], {'Form','Ideal','P',num2str(Kc),'I',num2str(1/Ti), ...
     'D',num2str(Td),'N',num2str(N/Td)});
% Planta
blk(m, 'Gp1',  'simulink/Continuous/Transfer Fcn', [330 140], {'Numerator','[1.2]','Denominator','[30 1]'});
blk(m, 'Lp1',  'simulink/Continuous/Transport Delay', [440 140], {'DelayTime','10'});
blk(m, 'Gd1',  'simulink/Continuous/Transfer Fcn', [330 220], {'Numerator','[0.9]','Denominator','[70 1]'});
blk(m, 'Ld1',  'simulink/Continuous/Transport Delay', [440 220], {'DelayTime','5'});
blk(m, 'SXD',  'simulink/Math Operations/Sum', [540 180], {'Inputs','++','IconShape','round'});
blk(m, 'Gp2',  'simulink/Continuous/Transfer Fcn', [330 300], {'Numerator','[1]','Denominator','[20 1]'});
blk(m, 'Gd2',  'simulink/Continuous/Transfer Fcn', [330 380], {'Numerator','[0.2]','Denominator','[60 1]'});
blk(m, 'Ld2',  'simulink/Continuous/Transport Delay', [440 380], {'DelayTime','5'});
blk(m, 'ST',   'simulink/Math Operations/Sum', [540 340], {'Inputs','++','IconShape','round'});
% Estimador: XDest = Gp1 R - (Gd1/Gd2) Gp2 R + (Gd1/Gd2) T
blk(m, 'Mp1',  'simulink/Continuous/Transfer Fcn', [330 470], {'Numerator','[1.2]','Denominator','[30 1]'});
blk(m, 'MLp1', 'simulink/Continuous/Transport Delay', [440 470], {'DelayTime','10'});
blk(m, 'MrGp2','simulink/Continuous/Transfer Fcn', [330 550], {'Numerator','4.5*[60 1]','Denominator','conv([70 1],[20 1])'});
blk(m, 'Mr',   'simulink/Continuous/Transfer Fcn', [640 340], {'Numerator','4.5*[60 1]','Denominator','[70 1]'});
blk(m, 'SEst', 'simulink/Math Operations/Sum', [760 470], {'Inputs','+-+','IconShape','round'});
blk(m, 'XD',   'simulink/Sinks/Out1', [660 180], {});
blk(m, 'XDest','simulink/Sinks/Out1', [860 470], {});
blk(m, 'R',    'simulink/Sinks/Out1', [330 30], {});
lin(m,'Ref/1','Suma/1'); lin(m,'Suma/1','PID/1');
lin(m,'PID/1','Gp1/1'); lin(m,'Gp1/1','Lp1/1'); lin(m,'Lp1/1','SXD/1');
lin(m,'XF/1','Gd1/1');  lin(m,'Gd1/1','Ld1/1'); lin(m,'Ld1/1','SXD/2'); lin(m,'SXD/1','XD/1');
lin(m,'PID/1','Gp2/1'); lin(m,'Gp2/1','ST/1');  lin(m,'XF/1','Gd2/1'); lin(m,'Gd2/1','Ld2/1'); lin(m,'Ld2/1','ST/2');
lin(m,'PID/1','Mp1/1'); lin(m,'Mp1/1','MLp1/1'); lin(m,'MLp1/1','SEst/1');
lin(m,'PID/1','MrGp2/1'); lin(m,'MrGp2/1','SEst/2'); lin(m,'ST/1','Mr/1'); lin(m,'Mr/1','SEst/3');
lin(m,'SEst/1','XDest/1'); lin(m,'SEst/1','Suma/2'); lin(m,'PID/1','R/1');
set_param(m, 'StopTime','800', 'MaxStep','0.1');  save_system(m);
o = sim(m);  y = o.yout;
t = y{1}.Values.Time;  xd = y{1}.Values.Data;  xde = y{2}.Values.Data;  R = y{3}.Values.Data;
f = figure('Position',[100 100 800 550]);
subplot(2,1,1); plot(t, xd, t, xde, '--', t, double(t >= 10), 'k:'); grid on; ylabel('X_D');
legend('X_D real','X_D inferida','referencia','Location','southeast');
title('Simulink cp6\_ej5: control inferencial (r = 1 en t = 10, X_F = 1 en t = 400)');
subplot(2,1,2); plot(t, R); grid on; ylabel('reflujo R'); xlabel('t [min]');
guardar(f, 'ej5_simulink.png');
fprintf('Ej5 Simulink: Mp = %.2f %%, XD(399) = %.4f, XD(800) = %.4f\n', (max(xd(t<400))-1)*100, interp1(t,xd,399), xd(end));
close_system(m, 0);

%% ======================== funciones ========================
function nuevo(m)
    if bdIsLoaded(m), close_system(m, 0); end
    if exist([m '.slx'], 'file'), delete([m '.slx']); end
    new_system(m);  set_param(m, 'SaveOutput','on', 'SaveFormat','Dataset', 'OutputSaveName','yout');
end
function blk(m, nombre, lib, pos, par)
    add_block(lib, [m '/' nombre], 'Position', [pos(1) pos(2)-18 pos(1)+70 pos(2)+18], par{:});
end
function lin(m, a, b)
    add_line(m, a, b, 'autorouting', 'smart');
end
function guardar(f, nombre)
    try, theme(f, 'light'); end
    exportgraphics(f, nombre, 'Resolution', 110);
end
