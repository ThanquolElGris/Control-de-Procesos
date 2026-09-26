function c = sintonia(metodo, tipo, varargin)
%SINTONIA  Reglas de sintonía de la Conf. 2. Devuelve un struct con Kc, Ti, Td
%          (forma IDEAL) y P, I, D (forma PARALELA), más la forma original.
%
%   c = sintonia('ZN-LC', 'PID', Ku, Tu)          % ZN lazo cerrado  (tabla para PID SERIE)
%   c = sintonia('ZN-LA', 'PI',  K, T, L)         % ZN lazo abierto  (tabla para PID SERIE)
%   c = sintonia('CC',    'PID', K, T, L)         % Cohen-Coon       (PID IDEAL)
%   c = sintonia('IAE-ref',  'PID', K, T, L)      % Rovira, cambios en la referencia (IDEAL)
%   c = sintonia('ITAE-ref', 'PI',  K, T, L)
%   c = sintonia('ISE-pert', 'PID', K, T, L)      % Smith-Murrill, perturbación (IDEAL)
%   c = sintonia('IAE-pert', 'P',   K, T, L)
%   c = sintonia('ITAE-pert','PID', K, T, L)
%   tipo: 'P', 'PI' o 'PID'
tipo = upper(tipo);  Td = 0; Ti = Inf; serie = false;
switch upper(metodo)
  case 'ZN-LC'
    [Ku, Tu] = deal(varargin{:}); serie = true;
    switch tipo
      case 'P',   Kc = Ku/2;
      case 'PI',  Kc = Ku/2.2; Ti = Tu/1.2;
      case 'PID', Kc = Ku/1.6; Ti = Tu/2;  Td = Tu/8;
    end
  case 'ZN-LA'
    [K, T, L] = deal(varargin{:}); serie = true;
    switch tipo
      case 'P',   Kc = T/(K*L);
      case 'PI',  Kc = 0.9*T/(K*L); Ti = 3.33*L;
      case 'PID', Kc = 1.2*T/(K*L); Ti = 2*L;  Td = 0.5*L;
    end
  case 'CC'
    [K, T, L] = deal(varargin{:}); r = L/T;
    switch tipo
      case 'P',   Kc = T/(K*L)*(1 + r/3);
      case 'PI',  Kc = T/(K*L)*(0.9 + r/12); Ti = L*(30 + 3*r)/(9 + 20*r);
      case 'PID', Kc = T/(K*L)*(4/3 + r/4);  Ti = L*(32 + 6*r)/(13 + 8*r); Td = 4*L/(11 + 2*r);
    end
  otherwise   % criterios integrales: 'ISE-pert','IAE-pert','ITAE-pert','IAE-ref','ITAE-ref'
    [K, T, L] = deal(varargin{:}); r = L/T;
    tab = tabla_ci(upper(metodo), tipo);
    Kc = tab(1)/K * r^tab(2);
    if contains(upper(metodo), 'REF')
      if numel(tab) >= 4, Ti = T/(tab(3) + tab(4)*r); end     % Rovira
    else
      if numel(tab) >= 4, Ti = T/tab(3) * r^tab(4); end       % Smith-Murrill
    end
    if numel(tab) == 6, Td = tab(5)*T*r^tab(6); end
end
c.metodo = metodo; c.tipo = tipo;
if serie                                   % pasar de serie a ideal
  c.serie = [Kc Ti Td];
  if Td > 0, Kc = Kc*(1 + Td/Ti); [Ti, Td] = deal(Ti + Td, Ti*Td/(Ti + Td)); end
end
c.Kc = Kc; c.Ti = Ti; c.Td = Td;
c.P = Kc; c.I = Kc/Ti; c.D = Kc*Td;        % paralela (I = 0 si Ti = Inf)
fprintf('%-10s %-3s  ideal: Kc=%.4g Ti=%.4g Td=%.4g  | paralela: P=%.4g I=%.4g D=%.4g\n', ...
        metodo, tipo, c.Kc, c.Ti, c.Td, c.P, c.I, c.D);
end

function tab = tabla_ci(met, tipo)
% [a1 b1 a2 b2 a3 b3] según la Conf. 2
switch [met '-' tipo]
  case 'ISE-PERT-P',    tab = [1.411 -0.917];
  case 'IAE-PERT-P',    tab = [0.902 -0.985];
  case 'ITAE-PERT-P',   tab = [0.490 -1.084];
  case 'ISE-PERT-PI',   tab = [1.305 -0.954 0.492 0.739];
  case 'IAE-PERT-PI',   tab = [0.984 -0.986 0.608 0.707];
  case 'ITAE-PERT-PI',  tab = [0.859 -0.977 0.674 0.680];
  case 'ISE-PERT-PID',  tab = [1.495 -0.945 1.101 0.771 0.560 1.006];
  case 'IAE-PERT-PID',  tab = [1.435 -0.921 0.878 0.749 0.482 1.137];
  case 'ITAE-PERT-PID', tab = [1.357 -0.947 0.842 0.738 0.381 0.995];
  case 'IAE-REF-PI',    tab = [0.758 -0.861 1.020 -0.323];
  case 'ITAE-REF-PI',   tab = [0.586 -0.916 1.030 -0.165];
  case 'IAE-REF-PID',   tab = [1.086 -0.869 0.740 -0.130 0.348 0.914];
  case 'ITAE-REF-PID',  tab = [0.965 -0.855 0.796 -0.147 0.308 0.929];
  otherwise, error('Combinación no tabulada: %s %s (para la referencia no hay P ni ISE).', met, tipo);
end
end
