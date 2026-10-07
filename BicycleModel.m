classdef BicycleModel < handle
    % BicycleModel - Modelo cinemático bicicleta (referenciado no eixo traseiro)
    %
    % Estados: x, y (posição), theta (heading), v (velocidade)
    % Entradas: a (aceleração longitudinal), delta (ângulo de esterçamento)
    %
    % Referência: Rajamani, "Vehicle Dynamics and Control", cap. 2.

    properties
        L       % entre-eixos (wheelbase) [m]
        state   % [x; y; theta; v]
    end

    methods
        function obj = BicycleModel(wheelbase, x0, y0, theta0, v0)
            % Construtor. Valores padrão: wheelbase=2.5, estado inicial zero.
            if nargin < 1, wheelbase = 2.5; end
            if nargin < 2, x0 = 0; end
            if nargin < 3, y0 = 0; end
            if nargin < 4, theta0 = 0; end
            if nargin < 5, v0 = 0; end

            obj.L = wheelbase;
            obj.state = [x0; y0; theta0; v0];
        end

        function dstate = dynamics(obj, state, a, delta)
            % Equações cinemáticas (derivada do estado)
            theta = state(3);
            v = state(4);

            dx = v * cos(theta);
            dy = v * sin(theta);
            dtheta = (v / obj.L) * tan(delta);
            dv = a;

            dstate = [dx; dy; dtheta; dv];
        end

        function state = step(obj, a, delta, dt)
            % Avança um passo de integração via Runge-Kutta 4ª ordem (RK4)
            s = obj.state;
            k1 = obj.dynamics(s, a, delta);
            k2 = obj.dynamics(s + dt/2 * k1, a, delta);
            k3 = obj.dynamics(s + dt/2 * k2, a, delta);
            k4 = obj.dynamics(s + dt * k3, a, delta);

            obj.state = s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

            % Normaliza o heading para [-pi, pi]
            obj.state(3) = atan2(sin(obj.state(3)), cos(obj.state(3)));

            state = obj.state;
        end

        function val = x(obj)
            val = obj.state(1);
        end

        function val = y(obj)
            val = obj.state(2);
        end

        function val = theta(obj)
            val = obj.state(3);
        end

        function val = v(obj)
            val = obj.state(4);
        end
    end
end
