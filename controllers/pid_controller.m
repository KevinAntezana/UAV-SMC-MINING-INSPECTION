function u = pid_controller(x_state, traj, params)
    % Controlador en cascada simple (PD) para baseline
    z = x_state(7); z_dot = x_state(8);
    x = x_state(9); x_dot = x_state(10);
    y = x_state(11); y_dot = x_state(12);
    
    % Constantes PD (sintonización empírica básica)
    kp_z = 15; kd_z = 10;
    kp_x = 3; kd_x = 2.5;
    kp_y = 3; kd_y = 2.5;
    kp_att = 20; kd_att = 8;

    % Control Z
    ez = traj.z - z; ez_dot = traj.z_dot - z_dot;
    u1 = (params.m / (cos(x_state(1))*cos(x_state(3)) + 1e-3)) * (params.g + traj.z_ddot + kp_z*ez + kd_z*ez_dot);
    u1 = max(0, min(u1, 30)); % Saturación de motores

    % Control X, Y (Generación de comandos de actitud)
    ex = traj.x - x; ex_dot = traj.x_dot - x_dot;
    ux_d = (params.m / u1) * (traj.x_ddot + kp_x*ex + kd_x*ex_dot);
    
    ey = traj.y - y; ey_dot = traj.y_dot - y_dot;
    uy_d = (params.m / u1) * (traj.y_ddot + kp_y*ey + kd_y*ey_dot);

    % Inversión cinemática con aproximación de ángulos pequeños y saturación
    theta_d = asin(max(-0.5, min(0.5, ux_d)));
    phi_d = asin(max(-0.5, min(0.5, -uy_d)));
    psi_d = traj.psi;

    % Control de Actitud (Roll, Pitch, Yaw)
    e_phi = phi_d - x_state(1); e_phi_dot = 0 - x_state(2);
    u2 = (1/params.b1) * (kp_att*e_phi + kd_att*e_phi_dot);

    e_theta = theta_d - x_state(3); e_theta_dot = 0 - x_state(4);
    u3 = (1/params.b2) * (kp_att*e_theta + kd_att*e_theta_dot);

    e_psi = psi_d - x_state(5); e_psi_dot = 0 - x_state(6);
    u4 = (1/params.b3) * (kp_att*e_psi + kd_att*e_psi_dot);

    u = [u1; u2; u3; u4];
end