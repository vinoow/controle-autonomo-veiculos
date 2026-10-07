%% Roda apos simular integrated_vehicle_control.slx no Simulink
% Gera o grafico a partir dos dados logados (v_sim, x_sim, y_sim, t_sim),
% no mesmo estilo visual do dia 4, para comparar diretamente.

% Descobre onde este script está salvo e define o diretório de outputs lá
script_path = fileparts(mfilename('fullpath'));
if isempty(script_path)
    script_path = pwd; % Fallback caso seja rodado direto linha por linha
end
output_dir = fullfile(script_path, 'outputs');

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
% Reconstroi v_ref (degrau) so para plotar a referencia junto
v_ref_value = 15; t_step = 1.0;
vref_sim = v_ref_value * (t_sim >= t_step);

% Recria o caminho de referencia (mesmos parametros do build_integrated_model.m)
x_path = linspace(0, 260, 7000);
dy1 = 4.05; dy2 = 5.7;
z1 = (2.4/25)*(x_path - 27) - 1.2;
z2 = (2.4/21.95)*(x_path - 56.46) - 1.2;
y_path = (dy1/2)*(1+tanh(z1)) - (dy2/2)*(1+tanh(z2));

figure('Position', [100, 100, 1000, 450]);

subplot(1,2,1);
plot(x_path, y_path, '--', 'LineWidth', 1); hold on;
plot(x_sim, y_sim, 'LineWidth', 1.5);
legend('Referencia', 'Veiculo (Simulink)', 'Location', 'southeast');
title('Trajetoria - modelo Simulink integrado');
xlabel('x [m]'); ylabel('y [m]'); axis equal; grid on;

subplot(1,2,2);
plot(t_sim, v_sim, 'LineWidth', 1.5); hold on;
plot(t_sim, vref_sim, '--', 'LineWidth', 1);
legend('v (Simulink)', 'v_{ref}', 'Location', 'southeast');
title('Velocidade - modelo Simulink integrado');
xlabel('tempo [s]'); ylabel('v [m/s]'); grid on;

saveas(gcf, fullfile(output_dir, 'simulink_integrated.png'));
fprintf('Grafico salvo em %s\n', fullfile(output_dir, 'simulink_integrated.png'));
