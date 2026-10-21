function [P, I, D, nombre] = tf2pid(Gc)
%TF2PID  Lleva una FT de controlador a la forma P + I/s + D*s e identifica el tipo.
%   Sirve para los resultados de sintesis() (MO/MS), p.ej. (0.25s^2+0.3125s+0.09375)/s
[num, den] = tfdata(minreal(Gc), 'v');
num = num(find(abs(num) > 1e-12, 1):end);
den = den(find(abs(den) > 1e-12, 1):end);
if numel(den) == 2 && abs(den(2)) < 1e-12          % denominador c*s  -> tiene integrador
  if numel(num) > 3, error('No es PID: tiene más de un cero extra (¿doble integrador?).'); end
  n = [zeros(1, 3 - numel(num)) num]/den(1);  D = n(1); P = n(2); I = n(3);
elseif numel(den) == 1                               % denominador constante -> sin integrador
  if numel(num) > 2, error('No es PID.'); end
  n = [zeros(1, 2 - numel(num)) num]/den(1);  D = n(1); P = n(2); I = 0;
else
  error('No es un PID (denominador %s). Revise si es PI con doble integrador (MS en planta tipo 0).', mat2str(den));
end
tipos = {'P','PD','PI','PID'};
nombre = tipos{1 + (D ~= 0) + 2*(I ~= 0)};
if P == 0 && I ~= 0 && D == 0, nombre = 'I'; end
fprintf('%s:  P = %.4g   I = %.4g   D = %.4g\n', nombre, P, I, D);
end
