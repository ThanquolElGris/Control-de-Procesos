%% MAIN26  Certamen 1 2026: simulaciones y comparación de estrategias
% Ejecutar después de calculos26.m (o se ejecuta solo si falta sintonias26.m).
% Todas las simulaciones: referencia escalón en t = 0, perturbación escalón unitaria en t = 80 s, man limitado a ±2.
if ~exist('sintonias26.mat', 'file'), calculos26; end
load('sintonias26.mat');
if ~exist('graficar', 'var'), graficar = true; end
LIM = 2;
ref  = @(p, b, aw) pid26(p(1), p(2), p(3), b, LIM, aw);          % PID del lazo único / externo
inn  = @(aw) pid26(P1, P1/I1, 0, 1, LIM, aw);                      % PI interno (MO): P = 2, I = 1

%% 1) Estrategias candidatas (back-calculation, escalones unitarios)
cand = { ...
  'Único: PID ITAE-ref (b=1)',    'unico',   ref(A.ITAE_ref, 1, 'back'),  [];
  'Único: PID ITAE-pert (b=1)',   'unico',   ref(A.ITAE_pert, 1, 'back'), [];
  'Único: PID ITAE-pert (b=0)',   'unico',   ref(A.ITAE_pert, 0, 'back'), [];
  'Cascada: ext. ITAE-ref (b=1)', 'cascada', ref(B.ITAE_ref, 1, 'back'),  inn('back');
  'Cascada: ext. ITAE-pert (b=0)','cascada', ref(B.ITAE_pert, 0, 'back'), inn('back');
  'Cascada: ext. CC (b=0.5)',     'cascada', ref(B.CC, 0.5, 'back'),      inn('back')};
fprintf('\n%-32s %7s %6s %6s | %7s %7s %8s | %6s\n', 'Estrategia', 'Mp[%]', 'tr[s]', 'ts[s]', 'pico', 'ts_p[s]', 'IAE_p', '|u|max');
res = cell(size(cand,1), 1);
for k = 1:size(cand,1)
    [t, y, u, m2] = simular26(cand{k,2}, cand{k,3}, cand{k,4});
    m = metricas26(t, y, u); res{k} = struct('t', t, 'y', y, 'u', u, 'm2', m2);
    fprintf('%-32s %7.1f %6.2f %6.2f | %7.3f %7.2f %8.3f | %6.2f\n', cand{k,1}, m.Mp, m.tr, m.ts, m.pico, m.ts_pert, m.IAE_pert, m.umax);
end

%% 2) Anti-windup: ninguno, back-calculation y clamping (escalón de referencia 1 y 3)
disenos = { ...
  'Único ITAE-pert b=0',   'unico',   @(aw) ref(A.ITAE_pert, 0, aw), @(aw) [];
  'Único ITAE-pert b=1',   'unico',   @(aw) ref(A.ITAE_pert, 1, aw), @(aw) [];
  'Cascada ITAE-ref b=1',  'cascada', @(aw) ref(B.ITAE_ref, 1, aw),  @(aw) inn(aw);
  'Cascada ZN b=1',        'cascada', @(aw) ref(B.ZN, 1, aw),        @(aw) inn(aw)};
metodos = {'none', 'back', 'clamp'};
aw_res = struct();
fprintf('\n%-24s %4s %-6s %7s %6s %6s\n', 'Diseño', 'r', 'AW', 'Mp[%]', 'tr[s]', 'ts[s]');
for k = 1:size(disenos,1)
    for amp = [1 3]
        for j = 1:3
            aw = metodos{j};
            [t, y, u] = simular26(disenos{k,2}, disenos{k,3}(aw), disenos{k,4}(aw), amp);
            m = metricas26(t, y, u, amp);
            fprintf('%-24s %4.0f %-6s %7.1f %6.2f %6.2f\n', disenos{k,1}, amp, aw, m.Mp, m.tr, m.ts);
            aw_res.(sprintf('d%d_r%d_%s', k, amp, aw)) = struct('t', t, 'y', y, 'u', u);
        end
    end
end

%% 3) Gráficas
if graficar
    figure('Name', 'Estrategias');
    subplot(3,1,1); hold on; for k = 1:numel(res), plot(res{k}.t, res{k}.y); end
    yline(1, 'k--'); grid on; title('Salida: ref. 1 en t = 0, pert. 1 en t = 80 s'); legend(cand(:,1), 'Location', 'southeast');
    subplot(3,1,2); hold on; for k = 1:numel(res), plot(res{k}.t, res{k}.u); end
    yline(2, 'r:'); yline(-2, 'r:'); grid on; title('Mando man');
    subplot(3,1,3); hold on; for k = [3 4], plot(res{k}.t, res{k}.y); end
    xlim([75 120]); grid on; title('Zoom en la perturbación: diseños elegidos'); legend(cand([3 4],1)); xlabel('t [s]');

    figure('Name', 'Anti-windup (referencia 3)');
    for k = [1 3]
        subplot(2,2,1+(k>1)); hold on
        for j = 1:3, q = aw_res.(sprintf('d%d_r3_%s', k, metodos{j})); plot(q.t, q.y); end
        xlim([0 60]); grid on; title([disenos{k,1} ': salida (r = 3)']); legend(metodos);
        subplot(2,2,3+(k>1)); hold on
        for j = 1:3, q = aw_res.(sprintf('d%d_r3_%s', k, metodos{j})); plot(q.t, q.u); end
        xlim([0 60]); grid on; title('Mando man'); legend(metodos); xlabel('t [s]');
    end
end
