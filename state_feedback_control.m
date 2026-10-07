%% Controle longitudinal via realimentacao de estado
% com acao integral, projetado por alocacao de polos.
%
%
% Modelo em espaco de estados (ver derivacao completa no README):
%   x = [v; xi]        xi = integral do erro de velocidade
%   dv/dt  = (-b/m)*v + (1/m)*u
%   dxi/dt = v_ref - v
%
%   A = [-b/m  0]      B = [1/m]      Bref = [0]
%       [-1    0]          [ 0 ]              [1]
%
% Lei de controle: u = -K1*v - K2*xi  
%
% Ganhos obtidos por alocacao de polos (ver derivacao em README):
%   K1 = 2*zeta*wn*m - b
%   K2 = -m*wn^2
%
% b foi recalculado a partir da linearizacao do arrasto aerodinamico
% quadratico em torno de v0=20 m/s (ver README).
clear; clc; close all;

if ~exist('outputs', 'dir')
    mkdir('outputs');
end

%% Parametros do veiculo
m = 1500;     % massa [kg]
b = 16.2;     % arrasto linearizado em v0=20 m/s [N*s/m] (ver README)

%% Especificacao de desempenho -> ganhos por alocacao de polos
zeta = 0.7;
wn = 1.576;

K1 = 2*zeta*wn*m - b;
K2 = -m*wn^2;

fprintf('Ganhos calculados: K1 = %.2f, K2 = %.2f\n', K1, K2);

%% Simulacao
dt = 0.02;
duration = 15.0;
n = round(duration / dt);

v_ref_value = 20;
t_step = 1.0;

veh = BicycleModel(2.5, 0, 0, 0, 0);
xi = 0;  % estado integral, inicia zerado

v_hist = zeros(1, n);
vref_hist = zeros(1, n);
u_hist = zeros(1, n);
t_hist = zeros(1, n);

t = 0;
for k = 1:n
    if t < t_step
        v_ref = 0;
    else
        v_ref = v_ref_value;
    end

    v = veh.v();
    e = v_ref - v;

    % Lei de controle: realimentacao de estado completo
    u = -K1*v - K2*xi;

    % Atualiza o estado integral (Euler) ANTES de avancar v,
    % para manter consistencia com a definicao dxi/dt = v_ref - v
    xi = xi + e * dt;

    % Converte forca em aceleracao (mesmo modelo fisico do dia 2)
    a = (u - b*v) / m;
    delta = 0;

    s = veh.step(a, delta, dt);

    v_hist(k) = s(4);
    vref_hist(k) = v_ref;
    u_hist(k) = u;
    t_hist(k) = t;

    t = t + dt;
end

%% Metricas
idx_step = find(t_hist >= t_step, 1);
v_after_step = v_hist(idx_step:end);
t_after_step = t_hist(idx_step:end) - t_step;

overshoot_pct = max((max(v_after_step) - v_ref_value) / v_ref_value * 100, 0);

tol = 0.02 * v_ref_value;
settled_idx = find(abs(v_after_step - v_ref_value) > tol, 1, 'last');
if isempty(settled_idx)
    settling_time = 0;
else
    settling_time = t_after_step(min(settled_idx + 1, length(t_after_step)));
end

ss_error = abs(v_ref_value - v_hist(end));

fprintf('--- Metricas de desempenho ---\n');
fprintf('Overshoot: %.2f %% (teorico para zeta=%.2f: %.2f %%)\n', ...
    overshoot_pct, zeta, exp(-zeta*pi/sqrt(1-zeta^2))*100);
fprintf('Tempo de acomodacao (2%%): %.2f s (teorico: %.2f s)\n', ...
    settling_time, 4/(zeta*wn));
fprintf('Erro em regime permanente: %.4f m/s\n', ss_error);

%% Plot
figure('Position', [100, 100, 700, 450]);
plot(t_hist, v_hist, 'LineWidth', 1.5); hold on;
plot(t_hist, vref_hist, '--', 'LineWidth', 1.2);
legend('v (realimentacao de estado)', 'v_{ref}', 'Location', 'southeast');
title('Controle longitudinal por alocacao de polos (acao integral)');
xlabel('tempo [s]'); ylabel('velocidade [m/s]');
grid on;

saveas(gcf, 'outputs/state_feedback_response.png');
fprintf('Grafico salvo em outputs/state_feedback_response.png\n');
