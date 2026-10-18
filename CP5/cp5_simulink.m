%% CP-5: construye, simula y dibuja los modelos Simulink
% Genera cp5_ej1.slx, cp5_ej2_simple.slx y cp5_ej2_cascada.slx, una imagen de cada diagrama
% (diag_*.png) y verifica que la simulación coincida con cp5.m.
% En cada modelo el bloque de ganancia "FF_on" activa (1) o desactiva (0) el feedforward.
% Disposición: perturbación y feedforward arriba, lazo principal al centro, realimentación abajo.
clear; clc; close all;
s = tf('s');

%% ======================== Ejercicio 1 ========================
m = 'cp5_ej1';  nuevo(m);
blk(m, 'Pert',  'simulink/Sources/Step', {'Time','10','Before','0','After','0.1'}, [0 -2]);
blk(m, 'Gff',   'simulink/Continuous/Transfer Fcn', {'Numerator','[-2 -4]','Denominator','[0.01 1]'}, [1.5 -1]);
blk(m, 'FF_on', 'simulink/Math Operations/Gain', {'Gain','1'}, [2.5 -1]);
blk(m, 'Kd',    'simulink/Math Operations/Gain', {'Gain','4'}, [4.4 -2]);
blk(m, 'Ref',   'simulink/Sources/Step', {'Time','1','Before','0','After','2'}, [0 0]);
blk(m, 'Sum_e', 'simulink/Math Operations/Sum', {'Inputs','|+-'}, [1 0]);
blk(m, 'Gc_MO', 'simulink/Continuous/Transfer Fcn', {'Numerator','[2 1]','Denominator','[3 0]'}, [2 0]);
blk(m, 'Sum_u', 'simulink/Math Operations/Sum', {'Inputs','++|'}, [3 0]);
blk(m, 'Gv',    'simulink/Continuous/Transfer Fcn', {'Numerator','2','Denominator','[1 2]'}, [4 0]);
blk(m, 'Sum_d', 'simulink/Math Operations/Sum', {'Inputs','++|'}, [5 0]);
blk(m, 'Gp',    'simulink/Continuous/Transfer Fcn', {'Numerator','6','Denominator','[4 2]'}, [6 0]);
sal(m, 'salida', [7.2 0]);  sal(m, 'u', [3.6 1]);  sal(m, 'ref', [0.6 1]);
lin(m,'Ref/1','Sum_e/1','r');   lin(m,'Sum_e/1','Gc_MO/1','e');  lin(m,'Gc_MO/1','Sum_u/2','u_fb');
lin(m,'Pert/1','Gff/1','d');    lin(m,'Gff/1','FF_on/1');        lin(m,'FF_on/1','Sum_u/1','u_ff');
lin(m,'Sum_u/1','Gv/1','u');    lin(m,'Gv/1','Sum_d/2');         lin(m,'Pert/1','Kd/1');
lin(m,'Kd/1','Sum_d/1');        lin(m,'Sum_d/1','Gp/1');         lin(m,'Gp/1','salida/1','y');
lin(m,'Gp/1','Sum_e/2');        lin(m,'Sum_u/1','u/1');          lin(m,'Ref/1','ref/1');
alcance(m, 'Ref/1', 'Gp/1', [7 -1.2]);
nota(m, {'CP5 Ej. 1: control por módulo óptimo (feedback) + feedforward de la perturbación.', ...
    'Feedback: Gc_MO = (2s+1)/(3s) es un PI cuyo cero cancela la constante lenta de Gp (T = 2); lazo cerrado con zeta = 0.707.', ...
    'Feedforward: se mide d y se actúa antes de que afecte la salida. Gff = -Gd/(Gv Gp) = -4/Gv = -4(0.5s+1) (impropia):', ...
    '   se implementa con un polo rápido, -4(0.5s+1)/(0.01s+1). Para el FF estático usar Gff = -4.', ...
    'FF_on = 1 activa el feedforward y FF_on = 0 lo desactiva (solo feedback). El FF no cambia la respuesta a la referencia.', ...
    'Experimento: r = 2 en t = 1 y d = 0.1 en t = 10. Pico ante d: 0.39 solo FB, 0.15 FF estático, 0.006 FF dinámico.'});
terminar(m, '20', '0.01');
for ff = {'0','1'}
    set_param([m '/FF_on'], 'Gain', ff{1});  o = sim(m);  [t,y] = senal(o,1);
    fprintf('Ej1 FF_on = %s: pico ante d = %+.4f\n', ff{1}, max(y(t>=10))-2);
end
set_param([m '/FF_on'], 'Gain', '1');  save_system(m);  close_system(m, 0);

%% ======================== Ejercicio 2 ========================
Gv = 2/(s+4);  G1 = 4/(4*s+2);  G2 = 6/(4*s+1);
Ga = Gv*G1*G2;  Ga.OutputDelay = 1;
[Ku,~,wu] = margin(Ga);  [Kc,Ti,Td] = zn_pid(Ku, 2*pi/wu);           % lazo simple
Gi = Gv*G1;  Gci = 4 + 2/s;
Gb = minreal(feedback(Gci*Gi,1))*G2;  Gb.OutputDelay = 1;
[Ku2,~,wu2] = margin(Gb);  [Kc2,Ti2,Td2] = zn_pid(Ku2, 2*pi/wu2);    % externo de la cascada

for caso = {'simple','cascada'}
    m = ['cp5_ej2_' caso{1}];  nuevo(m);
    c = 2*strcmp(caso{1}, 'cascada');            % la cascada agrega dos columnas (Sum_e2 y PI_int)
    blk(m, 'Ref',   'simulink/Sources/Step', {'Time','1','Before','0','After','2'}, [0 0]);
    blk(m, 'Sum_e1','simulink/Math Operations/Sum', {'Inputs','|+-'}, [1 0]);
    if c == 0
        blk(m, 'PID_ZN', 'simulink/Continuous/PID Controller', pid_ideal(Kc, Ti, Td), [2 0]);
    else
        blk(m, 'PID_ext', 'simulink/Continuous/PID Controller', pid_ideal(Kc2, Ti2, Td2), [2 0]);
        blk(m, 'Sum_e2',  'simulink/Math Operations/Sum', {'Inputs','|+-'}, [3 0]);
        blk(m, 'PI_int',  'simulink/Continuous/PID Controller', {'Controller','PI','Form','Parallel','P','4','I','2'}, [4 0]);
    end
    blk(m, 'Pert',  'simulink/Sources/Step', {'Time','30','Before','0','After','0.1'}, [0 -2]);
    blk(m, 'Gff',   'simulink/Continuous/Transfer Fcn', {'Numerator','[-1 -4]','Denominator','[1 10]'}, [c+1 -1]);
    blk(m, 'Lff',   'simulink/Continuous/Transport Delay', {'DelayTime','2'}, [c+1.9 -1]);
    blk(m, 'FF_on', 'simulink/Math Operations/Gain', {'Gain','1'}, [c+2.6 -1]);
    blk(m, 'Sum_u', 'simulink/Math Operations/Sum', {'Inputs','++|'}, [c+3 0]);
    blk(m, 'Gv',    'simulink/Continuous/Transfer Fcn', {'Numerator','2','Denominator','[1 4]'}, [c+4 0]);
    blk(m, 'Gd',    'simulink/Continuous/Transfer Fcn', {'Numerator','2','Denominator','[1 10]'}, [c+3.6 -2]);
    blk(m, 'Ld',    'simulink/Continuous/Transport Delay', {'DelayTime','2'}, [c+4.5 -2]);
    blk(m, 'Sum_d', 'simulink/Math Operations/Sum', {'Inputs','++|'}, [c+5 0]);
    blk(m, 'G1',    'simulink/Continuous/Transfer Fcn', {'Numerator','4','Denominator','[4 2]'}, [c+6 0]);
    blk(m, 'G2',    'simulink/Continuous/Transfer Fcn', {'Numerator','6','Denominator','[4 1]'}, [c+7 0]);
    blk(m, 'Lp',    'simulink/Continuous/Transport Delay', {'DelayTime','1'}, [c+8 0]);
    sal(m, 'salida1', [c+9 0]);  sal(m, 'u', [c+3.6 1]);  sal(m, 'salida2', [c+6.7 -1]);  sal(m, 'ref', [0.6 1]);
    lin(m,'Ref/1','Sum_e1/1','r');
    if c == 0
        lin(m,'Sum_e1/1','PID_ZN/1','e1');  lin(m,'PID_ZN/1','Sum_u/2','u_fb');
    else
        lin(m,'Sum_e1/1','PID_ext/1','e1'); lin(m,'PID_ext/1','Sum_e2/1','r2');
        lin(m,'Sum_e2/1','PI_int/1','e2');  lin(m,'PI_int/1','Sum_u/2','u_fb');
        lin(m,'G1/1','Sum_e2/2');
    end
    lin(m,'Pert/1','Gff/1','d');  lin(m,'Gff/1','Lff/1');  lin(m,'Lff/1','FF_on/1');  lin(m,'FF_on/1','Sum_u/1','u_ff');
    lin(m,'Sum_u/1','Gv/1','u');  lin(m,'Gv/1','Sum_d/2');  lin(m,'Pert/1','Gd/1');  lin(m,'Gd/1','Ld/1');
    lin(m,'Ld/1','Sum_d/1');      lin(m,'Sum_d/1','G1/1');  lin(m,'G1/1','G2/1');
    lin(m,'G2/1','Lp/1');         lin(m,'Lp/1','salida1/1','salida1');  lin(m,'Lp/1','Sum_e1/2');
    lin(m,'Sum_u/1','u/1');       lin(m,'G1/1','salida2/1');  lin(m,'Ref/1','ref/1');
    alcance(m, 'Ref/1', 'Lp/1', [c+9 -1.2]);
    if c == 0
        txt = {'CP5 Ej. 2 a), b) y d): lazo simple con PID por Ziegler-Nichols sobre salida1 (con el retardo Lp = 1).', ...
               'PID ideal (ZN serie convertido): Kc = 0.737, Ti = 5.32, Td = 0.851 (Ku = 0.943, Tu = 8.51). Mp ~ 55 % (típico de ZN).'};
    else
        txt = {'CP5 Ej. 2 c) y d): cascada. Lazo interno sobre salida2 (antes de G2 y del retardo) y lazo externo sobre salida1.', ...
               'Interno: PI por módulo óptimo 4 + 2/s (la planta interna no tiene retardo, así que no tiene Ku y ZN no sirve).', ...
               'Externo: PID por ZN sobre el lazo interno cerrado: Kc = 0.631, Ti = 3.36, Td = 0.538.', ...
               'La perturbación entra dentro del lazo interno: la cascada reduce su pico unas 5 veces respecto del lazo simple.'};
    end
    nota(m, [txt, {'Feedforward: Gff = -Gd e^{-2s}/Gv = -(s+4)/(s+10) e^{-2s}. Es propio y con retardo positivo: realizable exacto.', ...
                   'FF_on = 1/0 activa o desactiva el FF (se suma al mando de la válvula). Con FF la perturbación prácticamente desaparece.', ...
                   'Experimento: r = 2 en t = 1 y d = 0.1 en t = 30. Para ver solo la perturbación, poner Final value = 0 en Ref.'}]);
    terminar(m, '60', '0.01');
    for ff = {'0','1'}
        set_param([m '/FF_on'], 'Gain', ff{1});  o = sim(m);  [t,y] = senal(o,1);
        Mp = (max(y(t >= 1 & t < 30))-2)/2*100;
        set_param([m '/Ref'], 'After', '0');  o = sim(m);  [~,y] = senal(o,1);   % solo perturbación (como en el README)
        set_param([m '/Ref'], 'After', '2');
        fprintf('Ej2 %-8s FF_on = %s: Mp = %5.1f %%  pico ante d (solo d) = %+.5f\n', caso{1}, ff{1}, Mp, y(find(abs(y)==max(abs(y)),1)));
    end
    set_param([m '/FF_on'], 'Gain', '1');  save_system(m);  close_system(m, 0);
end

%% ======================== funciones ========================
function nota(m, lineas)
    % Nota de texto explicativa debajo del diagrama (queda guardada en el .slx y sale en la imagen)
    b = find_system(m, 'SearchDepth', 1, 'Type', 'Block');  p = cell2mat(get_param(b, 'Position'));
    a = Simulink.Annotation(m, strjoin(lineas, newline));
    a.Position = [min(p(:,1)) max(p(:,4))+50 min(p(:,1))+10 max(p(:,4))+60];
    a.HorizontalAlignment = 'left';  a.BackgroundColor = '[1 1 0.88]';  a.FontSize = 11;
end
function [Kc,Ti,Td] = zn_pid(Ku, Tu)
    Kcs = Ku/1.6;  Tis = Tu/2;  Tds = Tu/8;
    Kc = Kcs*(1+Tds/Tis);  Ti = Tis+Tds;  Td = Tis*Tds/(Tis+Tds);
end
function p = pid_ideal(Kc, Ti, Td)
    % PID ideal Kc(1 + 1/(Ti s) + Td s/((Td/10) s + 1)) en el bloque PID Controller (Form = Ideal)
    p = {'Form','Ideal','P',sprintf('%.4f',Kc),'I',sprintf('%.4f',1/Ti),'D',sprintf('%.4f',Td),'N',sprintf('%.4f',10/Td)};
end
function nuevo(m)
    if bdIsLoaded(m), close_system(m, 0); end
    if exist([m '.slx'], 'file'), delete([m '.slx']); end
    new_system(m);  set_param(m, 'SaveOutput','on', 'SaveFormat','Dataset', 'OutputSaveName','yout');
end
function p = pos(cr, w, h)
    % columna/fila de la grilla -> rectángulo [izq arriba der abajo]
    x = 60 + cr(1)*170;  y = 220 + cr(2)*80;
    p = round([x-w/2 y-h/2 x+w/2 y+h/2]);
end
function blk(m, nombre, lib, par, cr)
    if contains(lib, {'Transfer Fcn','PID'}),   w = 120; h = 46;
    elseif contains(lib, 'Sum'),                 w = 30;  h = 30;  par = [par {'IconShape','round'}];
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
    % Mux [referencia; salida] -> Scope, para ver la respuesta al abrir el modelo
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
