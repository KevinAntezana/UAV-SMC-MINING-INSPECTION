function plot_results(T, X, U, Traj, title_str)
    figure('Name', title_str, 'Color', 'w', 'Position', [100, 100, 1000, 600]);

    % Trayectoria 3D y Entorno Minero
    subplot(2,2,[1 3]);
    
    % Dibujar Chimenea Vertical (Radio 1m, Altura de Z=0 a Z=10)
    [xc, yc, zc] = cylinder(1.0, 30);
    surf(xc, yc, zc * 10, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 0.5]); hold on;
    
    % Dibujar Galería Inferior (Túnel en X de -4m a 1m, centro en Z=1m)
    surf(zc*5 - 4, xc, yc + 1, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.8 0.6 0.4]);
    
    % Dibujar Galería Superior (Túnel en X de -1m a 4m, centro en Z=10m)
    surf(zc*5 - 1, xc, yc + 10, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.8 0.6 0.4]);

    % Dibujar Trayectorias
    plot3(Traj(1,:), Traj(2,:), Traj(3,:), 'r--', 'LineWidth', 1.5);
    plot3(X(9,:), X(11,:), X(7,:), 'b', 'LineWidth', 1.5); 
    
    grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Trayectoria de Inspección en Galerías y Chimenea');
    legend('Chimenea', 'Galerías', '', 'Referencia', 'Vuelo Real UAV', 'Location', 'best');
    view(-35, 20); % Ángulo de cámara optimizado para ver el perfil en "Z"

    % Errores XYZ
    subplot(2,2,2);
    plot(T, X(9,:) - Traj(1,:), 'r', T, X(11,:) - Traj(2,:), 'g', T, X(7,:) - Traj(3,:), 'b');
    grid on; xlabel('Tiempo [s]'); ylabel('Error [m]');
    title('Errores de Seguimiento (X, Y, Z)'); legend('e_x', 'e_y', 'e_z');

    % Acciones de Control
    subplot(2,2,4);
    plot(T(1:end-1), U(1,:), 'k', 'LineWidth', 1.2);
    grid on; xlabel('Tiempo [s]'); ylabel('Empuje Total [N]');
    title('Señal de Control (Thrust U1)');
end