function c = pid26(Kc, Ti, Td, b, lim, aw, N)
% PID26  Arma el struct del PID a partir de la forma ideal (Kc, Ti, Td).
%   b: peso de la referencia en la acción P (c = 0: D sobre la medición).
if nargin < 4, b = 1; end
if nargin < 5, lim = inf; end
if nargin < 6, aw = 'back'; end
if nargin < 7, N = 10; end
c.P = Kc; c.I = Kc/Ti; c.D = Kc*Td; c.N = N; c.b = b; c.c = 0; c.lim = lim; c.aw = aw;
if Td > 0, c.Kb = 1/sqrt(Ti*Td); else, c.Kb = 1/Ti; end
end
