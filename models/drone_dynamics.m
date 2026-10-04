function dx = drone_dynamics(t, x_state, u, params, dist)
    % Extraer estados (Modelo de 12 estados)
    phi = x_state(1);       phi_dot = x_state(2);
    theta = x_state(3);     theta_dot = x_state(4);
    psi = x_state(5);       psi_dot = x_state(6);
    z = x_state(7);         z_dot = x_state(8);
    x = x_state(9);         x_dot = x_state(10);
    y = x_state(11);        y_dot = x_state(12);

    u1 = u(1); u2 = u(2); u3 = u(3); u4 = u(4);

    % Factores trigonométricos
    ut = cos(phi)*cos(theta);
    ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);

    % Inicializar vector de derivadas
    dx = zeros(12,1);

    % Dinámica rotacional
    dx(1) = phi_dot;
    dx(2) = theta_dot * psi_dot * params.a1 + params.b1 * u2;
    dx(3) = theta_dot;
    dx(4) = phi_dot * psi_dot * params.a2 + params.b2 * u3;
    dx(5) = psi_dot;
    dx(6) = phi_dot * theta_dot * params.a3 + params.b3 * u4;

    % Dinámica traslacional (incluyendo perturbaciones aerodinámicas)
    dx(7) = z_dot;
    dx(8) = -params.g + (ut * u1 / params.m) + dist.z;
    dx(9) = x_dot;
    dx(10) = (ux * u1 / params.m) + dist.x;
    dx(11) = y_dot;
    dx(12) = (uy * u1 / params.m) + dist.y;
end