function dist = disturbances(x_state, t)
    % Parámetros de la chimenea minera
    R_chimney = 1.0; % Radio de la chimenea en metros
    
    x = x_state(9); 
    y = x_state(11);
    r = sqrt(x^2 + y^2);

    % Interacción con las paredes (Efecto suelo/pared repulsivo)
    wall_repulsion = 0;
    if r > R_chimney * 0.8
        % Aumento exponencial de la fuerza repulsiva al acercarse a la pared
        wall_repulsion = -8 * (r - R_chimney*0.8)^2; 
    end

    dir_x = x / (r + 1e-3);
    dir_y = y / (r + 1e-3);

    % Corrientes de aire térmicas (vórtices y flujos ascendentes)
    wind_x = 0.3 * sin(1.2 * t) + 0.1 * randn;
    wind_y = 0.3 * cos(0.8 * t) + 0.1 * randn;
    wind_z = 0.2 * sin(0.5 * t); % Turbulencia vertical

    % Acumulación de perturbaciones en aceleración
    dist.x = wind_x + wall_repulsion * dir_x;
    dist.y = wind_y + wall_repulsion * dir_y;
    dist.z = wind_z;
end