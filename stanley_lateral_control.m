    %% Controle lateral (Stanley Controller)
    %
    % Testa o controlador numa manobra classica de dupla mudanca de faixa
    % (double lane change, baseada na norma ISO 3888), usada na industria
    % automotiva para avaliar estabilidade e capacidade de manobra evasiva.
    %
    % Lei de controle de Stanley:
    %   delta = theta_e + atan2(k*e, v)
    %
    % onde theta_e = erro de heading (direcao do caminho - direcao do carro)
    %      e       = erro lateral (distancia do eixo dianteiro ao caminho)
    %      k       = ganho de correcao lateral
    %
    % Velocidade mantida constante.
    clear; clc; close all;
    
    if ~exist('outputs', 'dir')
        mkdir('outputs');
    end
    
    %% Gera o caminho de referencia: dupla mudanca de faixa (formula classica
    % usada em papers de path-tracking, ex: Hoffmann et al. 2007)
    x_path = linspace(0, 100, 4000);
    
    dy1 = 4.05; dy2 = 5.7;
    z1 = (2.4/25)*(x_path - 27) - 1.2;
    z2 = (2.4/21.95)*(x_path - 56.46) - 1.2;
    y_path = (dy1/2)*(1+tanh(z1)) - (dy2/2)*(1+tanh(z2));
    
    % Heading do caminho em cada ponto (derivada numerica)
    theta_path = atan2(diff(y_path), diff(x_path));
    theta_path(end+1) = theta_path(end);  % mesmo tamanho do vetor
    
    %% Parametros do veiculo e do controlador
    L = 2.5;        % wheelbase [m]
    v_const = 15;   % velocidade constante para este teste [m/s]
    k_gain = 1.5;   % ganho de correcao lateral (Stanley)
    k_soft = 1.0;   % termo de suavizacao (evita instabilidade em v baixo)
    delta_max = deg2rad(30);  % limite fisico de esterçamento
    
    %% Simulacao
    dt = 0.02;
    duration = 8.0;  % tempo para percorrer os 100m a 15 m/s (~6.7s) + folga
    n = round(duration / dt);
    
    veh = BicycleModel(L, 0, -2.2, deg2rad(29), v_const);  % pequeno erro lateral inicial
    
    x_hist = zeros(1, n); y_hist = zeros(1, n);
    e_hist = zeros(1, n); delta_hist = zeros(1, n); t_hist = zeros(1, n);
    
    t = 0;
    for k_idx = 1:n
        % Posicao do eixo dianteiro (Stanley usa o eixo dianteiro, nao o traseiro)
        xf = veh.x() + L*cos(veh.theta());
        yf = veh.y() + L*sin(veh.theta());
    
        % Encontra o ponto mais proximo no caminho de referencia
        dists = (x_path - xf).^2 + (y_path - yf).^2;
        [~, idx] = min(dists);
    
        % Erro lateral (cross-track), projetado no eixo dianteiro do VEICULO
        % (nao no heading do caminho - esse foi o bug da primeira tentativa,
        % que causava realimentacao positiva em vez de negativa nas curvas)
        dx = xf - x_path(idx);
        dy = yf - y_path(idx);
        theta_v = veh.theta();
        e = dx*sin(theta_v) - dy*cos(theta_v);
    
        % Erro de heading, normalizado para [-pi, pi]
        theta_e = theta_path(idx) - veh.theta();
        theta_e = atan2(sin(theta_e), cos(theta_e));
    
        % Lei de controle de Stanley
        delta = theta_e + atan2(k_gain*e, v_const + k_soft);
        delta = max(min(delta, delta_max), -delta_max);  % saturacao fisica
    
        % Avanca o modelo (aceleracao=0 -> velocidade constante)
        s = veh.step(0, delta, dt);
    
        x_hist(k_idx) = s(1); y_hist(k_idx) = s(2);
        e_hist(k_idx) = e; delta_hist(k_idx) = delta; t_hist(k_idx) = t;
        t = t + dt;
    end
    
    %% Metricas
    max_abs_error = max(abs(e_hist));
    fprintf('Erro lateral maximo: %.3f m\n', max_abs_error);
    fprintf('Erro lateral final: %.4f m\n', e_hist(end));
    
    %% Plots
    figure('Position', [100, 100, 1100, 450]);
    
    subplot(1, 2, 1);
    plot(x_path, y_path, '--', 'LineWidth', 1.2); hold on;
    plot(x_hist, y_hist, 'LineWidth', 1.5);
    legend('Caminho de referencia', 'Trajetoria do veiculo', 'Location', 'southeast');
    title('Manobra de dupla mudanca de faixa (ISO 3888)');
    xlabel('x [m]'); ylabel('y [m]');
    axis equal; grid on;
    
    subplot(1, 2, 2);
    plot(t_hist, e_hist, 'LineWidth', 1.5);
    title(sprintf('Erro lateral (max = %.2f m)', max_abs_error));
    xlabel('tempo [s]'); ylabel('erro lateral [m]');
    grid on;
    
    saveas(gcf, 'outputs/stanley_lane_change.png');
    fprintf('Grafico salvo em outputs/stanley_lane_change.png\n');
