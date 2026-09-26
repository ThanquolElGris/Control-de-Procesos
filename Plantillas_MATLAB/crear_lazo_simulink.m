function crear_lazo_simulink(nombre, planta_num, planta_den, P, I, D, N, lim, Kb, L)
%CREAR_LAZO_SIMULINK  (OPCIONAL) Crea por código un modelo Simulink de lazo simple:
%   Step(ref) -> Sum -> PID Controller (paralelo, filtro N, saturación, antiwindup)
%   -> Sum(+pert) -> Transfer Fcn planta -> [Transport Delay] -> salida realimentada,
%   con Scopes y To Workspace (y, u, e).
%   Ejemplo:  crear_lazo_simulink('lazo1', 5, conv([2 1],[0.5 1]), 1, 0.5, 0.2, 100, 3, 0.5, 0)
%   ADVERTENCIA: no probado en MATLAB (se escribió sin acceso a MATLAB). Si falla,
%   armar el modelo a mano siguiendo el esquema de la guía (sección 3.3).
if nargin < 8 || isempty(lim), lim = Inf; end
if nargin < 9 || isempty(Kb),  Kb = 0;  end
if nargin < 10 || isempty(L),  L = 0;   end
if bdIsLoaded(nombre), close_system(nombre, 0); end
new_system(nombre); open_system(nombre);
b = @(x) [nombre '/' x];
add_block('simulink/Sources/Step', b('Referencia'), 'Time', '1', 'Before', '0', 'After', '1', 'Position', [30 80 60 110]);
add_block('simulink/Sources/Step', b('Perturbacion'), 'Time', '50', 'Before', '0', 'After', '0', 'Position', [330 10 360 40]);
add_block('simulink/Math Operations/Sum', b('Error'), 'Inputs', '|+-', 'Position', [100 85 120 105]);
add_block('simulink/Continuous/PID Controller', b('PID'), 'Position', [160 75 220 115]);
set_param(b('PID'), 'Form', 'Parallel', 'P', num2str(P), 'I', num2str(I), 'D', num2str(D), 'N', num2str(N));
if isfinite(lim)
  set_param(b('PID'), 'LimitOutput', 'on', 'UpperSaturationLimit', num2str(lim), 'LowerSaturationLimit', num2str(-lim));
  if Kb > 0, set_param(b('PID'), 'AntiWindupMode', 'back-calculation', 'Kb', num2str(Kb)); end
end
add_block('simulink/Math Operations/Sum', b('SumPert'), 'Inputs', '++', 'Position', [260 85 280 105]);
add_block('simulink/Continuous/Transfer Fcn', b('Planta'), 'Numerator', mat2str(planta_num), ...
          'Denominator', mat2str(planta_den), 'Position', [320 75 400 115]);
add_block('simulink/Continuous/Transport Delay', b('Retardo'), 'DelayTime', num2str(L), 'Position', [430 80 470 110]);
add_block('simulink/Sinks/Scope', b('Scope_y'), 'Position', [560 80 590 110]);
add_block('simulink/Sinks/To Workspace', b('y_ws'), 'VariableName', 'y', 'SaveFormat', 'Array', 'Position', [560 130 620 150]);
add_block('simulink/Sinks/To Workspace', b('u_ws'), 'VariableName', 'u', 'SaveFormat', 'Array', 'Position', [260 150 320 170]);
add_block('simulink/Sinks/To Workspace', b('e_ws'), 'VariableName', 'e', 'SaveFormat', 'Array', 'Position', [160 150 220 170]);
add_line(nombre, 'Referencia/1', 'Error/1', 'autorouting', 'on');
add_line(nombre, 'Error/1', 'PID/1', 'autorouting', 'on');
add_line(nombre, 'Error/1', 'e_ws/1', 'autorouting', 'on');
add_line(nombre, 'PID/1', 'SumPert/1', 'autorouting', 'on');
add_line(nombre, 'PID/1', 'u_ws/1', 'autorouting', 'on');
add_line(nombre, 'Perturbacion/1', 'SumPert/2', 'autorouting', 'on');
add_line(nombre, 'SumPert/1', 'Planta/1', 'autorouting', 'on');
add_line(nombre, 'Planta/1', 'Retardo/1', 'autorouting', 'on');
add_line(nombre, 'Retardo/1', 'Scope_y/1', 'autorouting', 'on');
add_line(nombre, 'Retardo/1', 'y_ws/1', 'autorouting', 'on');
add_line(nombre, 'Retardo/1', 'Error/2', 'autorouting', 'on');
set_param(nombre, 'StopTime', '100', 'MaxStep', '0.01');
save_system(nombre);
fprintf('Modelo %s creado. Cambie la amplitud de "Perturbacion" (After) para aplicarla.\n', nombre);
end
