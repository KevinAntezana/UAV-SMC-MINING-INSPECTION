function [X_hat, P_new] = ekf_estimator(X_hat_prev, P_prev, u_prev, att_meas, z_meas, params, dt, Q, R)
    % EKF para estimación de Posición y Velocidad del UAV
    % Estados EKF: X_hat = [x, y, z, v_x, v_y, v_z]' (6x1)
    
    %% --- 1. PREDICCIÓN (Modelo Dinámico) ---
    % Descomponer actitud medida (IMU)
    phi = att_meas(1); theta = att_meas(2); psi = att_meas(3);
    u1 = u_prev(1); % Empuje total aplicado en el instante anterior
    
    % Factores trigonométricos
    ut = cos(phi)*cos(theta);
    ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    
    % Aceleraciones esperadas (Modelo cinemático)
    a_x = ux * u1 / params.m;
    a_y = uy * u1 / params.m;
    a_z = -params.g + (ut * u1 / params.m);
    
    % Actualización del estado apriori (Integración Euler)
    X_pred = zeros(6,1);
    X_pred(1:3) = X_hat_prev(1:3) + X_hat_prev(4:6) * dt;
    X_pred(4:6) = X_hat_prev(4:6) + [a_x; a_y; a_z] * dt;
    
    % Jacobiano del modelo de proceso (F)
    % F = I + A*dt
    F = eye(6);
    F(1,4) = dt; F(2,5) = dt; F(3,6) = dt;
    
    % Covarianza apriori
    P_pred = F * P_prev * F' + Q;
    
    %% --- 2. ACTUALIZACIÓN (Modelo de Medición) ---
    x_p = X_pred(1); y_p = X_pred(2); z_p = X_pred(3);
    
    % Modelo de medición esperado h(X_pred)
    % h = [x; y; z; R_chimney - sqrt(x^2 + y^2)]
    r_p = sqrt(x_p^2 + y_p^2);
    d_wall_pred = 1.0 - r_p;
    z_expected = [x_p; y_p; z_p; d_wall_pred];
    
    % Jacobiano del modelo de medición (H)
    H = zeros(4,6);
    % Derivadas de posición directa (SLAM)
    H(1,1) = 1; 
    H(2,2) = 1; 
    H(3,3) = 1;
    % Derivadas de distancia a la pared (Sensor proximidad)
    % Prevenir división por cero si el dron está exactamente en (0,0)
    if r_p < 1e-4
        H(4,1) = 0; H(4,2) = 0;
    else
        H(4,1) = -x_p / r_p; 
        H(4,2) = -y_p / r_p;
    end
    
    % --- Filtro de Kalman ---
    % Innovación
    y_res = z_meas - z_expected;
    
    % Covarianza de la innovación (S)
    S = H * P_pred * H' + R;
    
    % Ganancia de Kalman (K)
    K = P_pred * H' / S;
    
    % Actualización aposteriori
    X_hat = X_pred + K * y_res;
    P_new = (eye(6) - K * H) * P_pred;
end