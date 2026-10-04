function plot_slam_results(T, X_real, X_est, Traj)
    % Métricas RMSE
    err_x = X_real(9,:) - X_est(1,:);
    err_y = X_real(11,:) - X_est(2,:);
    err_z = X_real(7,:) - X_est(3,:);
    
    rmse_x = sqrt(mean(err_x.^2));
    rmse_y = sqrt(mean(err_y.^2));
    rmse_z = sqrt(mean(err_z.^2));
    
    fprintf('Métricas de Estimación EKF (RMSE):\n');
    fprintf('X: %.4f m | Y: %.4f m | Z: %.4f m\n', rmse_x, rmse_y, rmse_z);

    figure('Name', 'Resultados SLAM-EKF vs Realidad', 'Color', 'w', 'Position', [200, 200, 1000, 600]);

    % 1. Trayectorias (Real vs Estimado)
    subplot(2, 2, [1 3]);
    plot3(Traj(1,:), Traj(2,:), Traj(3,:), 'k--', 'LineWidth', 1.5); hold on;
    plot3(X_real(9,:), X_real(11,:), X_real(7,:), 'b', 'LineWidth', 1.5);
    plot3(X_est(1,:), X_est(2,:), X_est(3,:), 'r-.', 'LineWidth', 1.2);
    
    % Renderizar pared de la chimenea para contexto de seguridad
    % Dibujar Chimenea Vertical (Radio 1m, Altura de Z=0 a Z=10)
    [xc, yc, zc] = cylinder(1.0, 30);
    surf(xc, yc, zc * 10, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 0.5]); hold on;
    
    % Dibujar Galería Inferior (Túnel en X de -4m a 1m, centro en Z=1m)
    surf(zc*5 - 4, xc, yc + 1, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.8 0.6 0.4]);
    
    % Dibujar Galería Superior (Túnel en X de -1m a 4m, centro en Z=10m)
    surf(zc*5 - 1, xc, yc + 10, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.8 0.6 0.4]);
    
    grid on; axis equal; view(-35, 20);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Fusión Sensorial: Trayectoria Real vs Estimada');
    legend('Referencia', 'Vuelo Real', 'Estimación EKF', 'Paredes', 'Location', 'best');

    % 2. Error de Estimación
    subplot(2, 2, 2);
    plot(T, err_x, 'r', T, err_y, 'g', T, err_z, 'b');
    grid on; xlabel('Tiempo [s]'); ylabel('Error $x - \hat{x}$ [m]', 'Interpreter', 'latex');
    title(sprintf('Error de Estimación (RMSE_{Total} = %.3f m)', norm([rmse_x, rmse_y, rmse_z])));
    legend('Error X', 'Error Y', 'Error Z');

    % 3. Métrica de Seguridad (Distancia a la pared)
    subplot(2, 2, 4);
    r_real = sqrt(X_real(9,:).^2 + X_real(11,:).^2);
    safety_dist = 1.0 - r_real;
    
    plot(T, safety_dist, 'm', 'LineWidth', 1.5); hold on;
    yline(0, 'r--', 'Colisión', 'LineWidth', 1.5);
    yline(0.2, 'y--', 'Límite Seguro (0.2m)', 'LineWidth', 1.5);
    
    grid on; xlabel('Tiempo [s]'); ylabel('Distancia [m]');
    title('Proximidad Crítica a la Pared (Chimenea)');
    ylim([-0.1 1.2]);
end