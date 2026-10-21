function R = reglas26(Ku, Tu, K, T, L)
% REGLAS26  Sintonías ZN, CC e ITAE (forma ideal [Kc Ti Td]) a partir de Ku, Tu y el PORT.
r = L/T;
R.ZN        = [Ku/1.6*(1 + (Tu/8)/(Tu/2)), Tu/2 + Tu/8, (Tu/2)*(Tu/8)/(Tu/2 + Tu/8)];   % serie -> ideal
R.CC        = [T/(K*L)*(4/3 + r/4), L*(32+6*r)/(13+8*r), 4*L/(11+2*r)];
R.ITAE_ref  = [0.965/K*r^-0.855, T/(0.796 - 0.147*r), 0.308*T*r^0.929];
R.ITAE_pert = [1.357/K*r^-0.947, T/0.842*r^0.738, 0.381*T*r^0.995];
f = fieldnames(R);
for i = 1:numel(f)
    q = R.(f{i});
    fprintf('  %-9s Kc = %.4f Ti = %.4f Td = %.4f -> P = %.4f I = %.4f D = %.4f\n', f{i}, q, q(1), q(1)/q(2), q(1)*q(3));
end
end
