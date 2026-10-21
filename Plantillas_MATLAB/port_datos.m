function [K, T, L, t28, t63] = port_datos(t, y, dU, t0)
%PORT_DATOS  Modelo PORT K*exp(-L s)/(T s + 1) por el método de 28 % y 63 %.
%   [K,T,L] = port_datos(t, y, dU, t0)
%   t, y : respuesta al escalón (vectores; de step() o de Simulink/To Workspace)
%   dU   : tamaño del escalón aplicado a la entrada
%   t0   : instante en que se aplicó el escalón (0 si se usó step())
%   Ejemplo:  [y,t] = step(G, 0:0.01:100);  [K,T,L] = port_datos(t, y, 1, 0)
if nargin < 4, t0 = 0; end
t = t(:); y = y(:);
y0  = y(find(t >= t0, 1));          % valor antes del escalón
dY  = y(end) - y0;
K   = dY / dU;
t28 = t(find((y - y0)/dY >= 0.283, 1)) - t0;   % medidos DESDE el escalón
t63 = t(find((y - y0)/dY >= 0.632, 1)) - t0;
T   = 1.5*(t63 - t28);
L   = t63 - T;
fprintf('PORT: K = %.4g  t28 = %.4g  t63 = %.4g  T = %.4g  L = %.4g  L/T = %.3f\n', ...
        K, t28, t63, T, L, L/T);
end
