function [t, y, u, m2] = simular26(estr, ext, inn, amp_r, amp_d, t_d, tf, dt)
% SIMULAR26  Simulación del certamen 2026 (equivalente al modelo Simulink).
%   estr = 'unico'   : ext es el PID que actúa sobre med1 y entrega man.
%   estr = 'cascada' : ext (sobre med1) entrega la referencia de med2; inn (sobre med2) entrega man.
%   ext, inn: structs con campos P, I, D, N, b, c, lim, aw ('none' | 'back' | 'clamp'), Kb.
%   Referencia escalón amp_r en t = 0; perturbación escalón amp_d en t = t_d.
%   No usa toolboxes: todos los bloques son de 1er orden y los retardos son buffers.
if nargin < 4, amp_r = 1; end
if nargin < 5, amp_d = 1; end
if nargin < 6, t_d = 80; end
if nargin < 7, tf = 160; end
if nargin < 8, dt = 0.005; end
n = round(tf/dt); t = (0:n-1)'*dt; y = zeros(n,1); u = y; m2 = y;
x1 = 0; x2 = 0; x3 = 0; xs = 0; xp = 0;          % estados de G1, G2, G3, Gs, Gp
b3 = zeros(round(1.5/dt),1); bs = zeros(round(0.5/dt),1); bp = zeros(round(1/dt),1);
k3 = 1; ks = 1; kp = 1; med1 = 0; med2 = 0;
se = estado0(); si = estado0();
for k = 1:n
    r = amp_r; d = amp_d*(t(k) >= t_d);
    if strcmp(estr, 'unico')
        [uk, se] = pid_paso(ext, se, r, med1, dt);
    else
        [r2, se] = pid_paso(ext, se, r, med1, dt);
        [uk, si] = pid_paso(inn, si, r2, med2, dt);
    end
    % Planta (Euler): G1 = 4/(s+2), Gp = 1/(3s+2), G2 = 0.5/(2s+1), G3 = 2/(4s+1), Gs = 10/(s+10)
    dp = bp(kp); bp(kp) = xp; kp = mod(kp, numel(bp)) + 1;       % Gp -> retardo 1
    med2 = x2;
    y3 = b3(k3); b3(k3) = x3; k3 = mod(k3, numel(b3)) + 1;      % G3 -> retardo 1.5 = salida
    med1 = bs(ks); bs(ks) = xs; ks = mod(ks, numel(bs)) + 1;    % Gs -> retardo 0.5 = med1
    x1 = x1 + dt*(-2*x1 + 4*uk);
    xp = xp + dt*(-2*xp + d)/3;
    x2 = x2 + dt*(-x2 + 0.5*(x1 + dp))/2;                       % la perturbación entra antes de G2
    x3 = x3 + dt*(-x3 + 2*x2)/4;
    xs = xs + dt*(-10*xs + 10*y3);
    y(k) = y3; u(k) = uk; m2(k) = med2;
end
end

function s = estado0()
s.xi = 0; s.xf = 0;
end

function [us, s] = pid_paso(c, s, r, m, dt)
% PID 2DOF paralelo como el bloque de Simulink: u = P(b r - m) + I∫(r - m) + D N s/(s+N) (c r - m)
e  = r - m;
ud = 0;
if c.D ~= 0, ud = c.N*(c.D*(c.c*r - m) - s.xf); end
v  = c.P*(c.b*r - m) + s.xi + ud;
us = min(max(v, -c.lim), c.lim);
di = c.I*e;
switch c.aw
    case 'back'
        di = di + c.Kb*(us - v);                                  % back-calculation
    case 'clamp'
        if us ~= v && sign(di) == sign(v), di = 0; end           % clamping (integración condicional)
end
s.xi = s.xi + dt*di; s.xf = s.xf + dt*ud;
end
