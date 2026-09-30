function Gc = sintesis(metodo, Gp, Tu, Kr)
%SINTESIS  Controlador por módulo óptimo ('MO') o módulo simétrico ('MS').
%   Gc = sintesis('MO', Gp, Tu)      Tu = MENOR constante de tiempo (no compensada)
%   Gc = sintesis('MS', Gp, Tu, Kr)  Kr = ganancia de la realimentación (1 por defecto)
%   Gp NO puede tener retardo (se necesita su inversa).
if nargin < 4, Kr = 1; end
s = tf('s');
switch upper(metodo)
  case 'MO', Gc = minreal(1/(Kr*2*Tu*s*(Tu*s + 1)*Gp));
  case 'MS', Gc = minreal((4*Tu*s + 1)/(Kr*8*Tu^2*s^2*(Tu*s + 1)*Gp));
end
end
