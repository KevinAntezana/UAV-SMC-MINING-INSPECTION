function z_meas = lidar_model(x_real)
    % LiDAR SLAM Simplificado para Chimenea Minera
    % Entradas: x_real (estado real de 12 grados de libertad)
    % Salidas: z_meas = [x_slam; y_slam; z_slam; d_wall]
    
    % Parámetros del sensor
    R_chimney = 1.0; % Radio de la chimenea
    max_range = 5.0; % Rango máximo del LiDAR
    
    % Extracción de posición real
    x = x_real(9);
    y = x_real(11);
    z = x_real(7);
    
    % 1. Odometría LiDAR SLAM (Posición con ruido)
    % El SLAM tiene un error que suele crecer, aquí simulamos ruido gaussiano
    x_slam = x + 0.03 * randn;
    y_slam = y + 0.03 * randn;
    z_slam = z + 0.05 * randn; % Z suele ser menos preciso o apoyado por barómetro
    
    % 2. Sensor de proximidad a la pared (Medición No Lineal)
    % Calcula la distancia mínima desde el centro del dron hasta la pared del cilindro
    r_current = sqrt(x^2 + y^2);
    d_wall_real = R_chimney - r_current;
    
    % Saturación de rango y ruido del láser
    d_wall_meas = min(max_range, d_wall_real) + 0.01 * randn;
    
    % Vector de mediciones
    z_meas = [x_slam; y_slam; z_slam; d_wall_meas];
end