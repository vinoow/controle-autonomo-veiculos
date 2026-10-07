%% Carrega no workspace base todos os parametros que os blocos Simulink
% (Stanley_Controller, Longitudinal_Control, Vehicle_Plant) precisam
% encontrar para rodar. RODE ISSO SEMPRE QUE ABRIR UMA SESSAO NOVA DO
% MATLAB antes de simular integrated_vehicle_control, mesmo sem mudar
% nada no modelo - os valores aqui nao ficam salvos dentro do .slx.

m = 1500; b = 16.2; L = 2.5;

zeta = 0.7; wn = 1.576;
K1 = 2*zeta*wn*m - b;
K2 = -m*wn^2;

k_gain = 1.5; k_soft = 1.0;
delta_max = deg2rad(30);

x_path = linspace(0, 260, 7000);
dy1 = 4.05; dy2 = 5.7;
z1 = (2.4/25)*(x_path - 27) - 1.2;
z2 = (2.4/21.95)*(x_path - 56.46) - 1.2;
y_path = (dy1/2)*(1+tanh(z1)) - (dy2/2)*(1+tanh(z2));
theta_path = atan2(diff(y_path), diff(x_path));
theta_path(end+1) = theta_path(end);

v_ref_value = 15; t_step = 1.0; duration = 14.0;

fprintf('Parametros carregados no workspace base.\n');
