function [K, T, L] = port26(taus, Kg, Ld, interno)
% PORT26  Respuesta al escalón en lazo abierto (Euler) y método 28 % / 63 %.
%   taus: constantes de 1er orden en serie; Kg: ganancia; Ld: retardo;
%   interno = true antepone el lazo interno cerrado 2/(s^2+2s+2).
if nargin < 4, interno = false; end
dt = 0.001; t = (0:dt:80)'; x = zeros(numel(taus),1); z = [0 0]; y = zeros(size(t));
for k = 1:numel(t)
    v = 1;
    if interno, v = z(1); z = z + dt*[z(2), -2*z(2) - 2*z(1) + 2]; end
    for j = 1:numel(taus), x(j) = x(j) + dt*(v - x(j))/taus(j); v = x(j); end
    y(k) = Kg*v;
end
y = [zeros(round(Ld/dt),1); y(1:end-round(Ld/dt))];
K = y(end); t28 = t(find(y >= 0.283*K, 1)); t63 = t(find(y >= 0.632*K, 1));
T = 1.5*(t63 - t28); L = t63 - T;
end
