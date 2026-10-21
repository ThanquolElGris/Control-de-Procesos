function [Ku, Tu, wu] = ku_tu(G)
%KU_TU  Ganancia y período críticos (para Ziegler-Nichols en lazo cerrado).
%   G puede tener retardo:  G = tf(num, den, 'InputDelay', L)  o  G = ...*exp(-L*s)
%   Incluir válvula, proceso y sensor (todo el lazo sin el controlador).
[Gm, ~, wu] = margin(G);
Ku = Gm;                 % en veces (NO en dB)
Tu = 2*pi/wu;
fprintf('Ku = %.4g   wu = %.4g rad/t   Tu = %.4g\n', Ku, wu, Tu);
if isinf(Ku), warning('La fase nunca llega a -180°: no existe Ku (ZN lazo cerrado no aplica).'); end
end
