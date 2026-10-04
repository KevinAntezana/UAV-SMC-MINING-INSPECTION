function u = smc_controller(x_state, traj, params)
    % Extracción de estados
    phi = x_state(1); phi_dot = x_state(2);
    theta = x_state(3); theta_dot = x_state(4);
    psi = x_state(5); psi_dot = x_state(6);
    z = x_state(7); z_dot = x_state(8);
    x = x_state(9); x_dot = x_state(10);
    y = x_state(11); y_dot = x_state(12);

    % Parámetros SMC
    lam_z = 3; K_z = 8; Bd_z = 0.5;
    lam_pos = 2; K_pos = 3; Bd_pos = 0.5;
    lam_att = 6; K_att = 15; Bd_att = 0.2;

    % 1. Superficie de Altitud (Z)
    ez = z - traj.z; ez_dot = z_dot - traj.z_dot;
    sz = ez_dot + lam_z * ez;
    
    ut = cos(phi)*cos(theta) + 1e-3; % Prevenir singularidad
    u1 = (params.m / ut) * (params.g + traj.z_ddot - lam_z*ez_dot - K_z*sat(sz, Bd_z));
    u1 = max(1, min(u1, 30));

    % 2. Superficies de Posición (X, Y)
    ex = x - traj.x; ex_dot = x_dot - traj.x_dot;
    sx = ex_dot + lam_pos * ex;
    ux_d = (params.m / u1) * (traj.x_ddot - lam_pos*ex_dot - K_pos*sat(sx, Bd_pos));

    ey = y - traj.y; ey_dot = y_dot - traj.y_dot;
    sy = ey_dot + lam_pos * ey;
    uy_d = (params.m / u1) * (traj.y_ddot - lam_pos*ey_dot - K_pos*sat(sy, Bd_pos));

    % Ángulos deseados (aproximación para psi = 0)
    theta_d = asin(max(-0.5, min(0.5, ux_d)));
    phi_d = asin(max(-0.5, min(0.5, -uy_d)));
    psi_d = traj.psi;

    % 3. Superficies de Actitud
    e_phi = phi - phi_d; e_phi_dot = phi_dot;
    s_phi = e_phi_dot + lam_att * e_phi;
    u2 = (1/params.b1) * (-params.a1*theta_dot*psi_dot - lam_att*e_phi_dot - K_att*sat(s_phi, Bd_att));

    e_theta = theta - theta_d; e_theta_dot = theta_dot;
    s_theta = e_theta_dot + lam_att * e_theta;
    u3 = (1/params.b2) * (-params.a2*phi_dot*psi_dot - lam_att*e_theta_dot - K_att*sat(s_theta, Bd_att));

    e_psi = psi - psi_d; e_psi_dot = psi_dot;
    s_psi = e_psi_dot + lam_att * e_psi;
    u4 = (1/params.b3) * (-params.a3*phi_dot*theta_dot - lam_att*e_psi_dot - K_att*sat(s_psi, Bd_att));

    u = [u1; u2; u3; u4];
end

% Función de saturación para reducir el chattering
function val = sat(s, bd)
    val = max(-1, min(1, s/bd));
end