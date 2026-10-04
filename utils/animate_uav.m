function animate_uav(T, X, Traj, params, save_video)
    % Si no se indica el 5to parámetro, por defecto no graba video
    if nargin < 5
        save_video = false;
    end

    % Crear figura y maximizarla para pantalla completa
    hFig = figure('Name', 'Telemetría y Animación UAV - Proyecto HUALKANA', ...
                  'Color', 'w', 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
    
    % Diseño de la interfaz: 2 columnas
    tlo = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % --- 1. Espacio para la Animación 3D ---
    ax3D = nexttile(tlo, 1, [3 2]);
    hold(ax3D, 'on'); grid(ax3D, 'on'); axis(ax3D, 'equal');
    
    % Renderizar entorno minero
    [xc, yc, zc] = cylinder(1.0, 30);
    surf(ax3D, xc, yc, zc * 10, 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 0.5]);
    surf(ax3D, zc*5 - 4, xc, yc + 1, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', [0.8 0.6 0.4]);
    surf(ax3D, zc*5 - 1, xc, yc + 10, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', [0.8 0.6 0.4]);
    plot3(ax3D, Traj(1,:), Traj(2,:), Traj(3,:), 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');

    view(ax3D, -35, 20);
    xlabel(ax3D, 'X [m]'); ylabel(ax3D, 'Y [m]'); zlabel(ax3D, 'Z [m]');
    title(ax3D, 'Vuelo Animado en Entorno Confinado');

    % Objetos del dron
    arm_x = plot3(ax3D, [0 0], [0 0], [0 0], 'k', 'LineWidth', 2.5);
    arm_y = plot3(ax3D, [0 0], [0 0], [0 0], 'k', 'LineWidth', 2.5);
    rotor_f = scatter3(ax3D, 0, 0, 0, 100, 'r', 'filled'); 
    rotors = scatter3(ax3D, zeros(1,3), zeros(1,3), zeros(1,3), 80, 'b', 'filled');
    rastro = animatedline(ax3D, 'Color', 'b', 'LineStyle', ':', 'MaximumNumPoints', 500);

    % --- 2. Gráficas de Posición ---
    axPos = nexttile(tlo, 3);
    hold(axPos, 'on'); grid(axPos, 'on');
    lineX = animatedline(axPos, 'Color', 'r', 'DisplayName', 'x');
    lineY = animatedline(axPos, 'Color', 'g', 'DisplayName', 'y');
    lineZ = animatedline(axPos, 'Color', 'b', 'DisplayName', 'z');
    title(axPos, 'Posición Real [m]'); legend(axPos, 'show', 'Location', 'northeast');
    xlim(axPos, [0 T(end)]); ylim(axPos, [-4 11]);

    % --- 3. Gráficas de Error ---
    axErr = nexttile(tlo, 6);
    hold(axErr, 'on'); grid(axErr, 'on');
    errX = animatedline(axErr, 'Color', 'r');
    errY = animatedline(axErr, 'Color', 'g');
    errZ = animatedline(axErr, 'Color', 'b');
    title(axErr, 'Error de Seguimiento (m)');
    xlim(axErr, [0 T(end)]); ylim(axErr, [-0.5 0.5]);

    % --- 4. Actitud / Ángulos ---
    axAtt = nexttile(tlo, 9);
    hold(axAtt, 'on'); grid(axAtt, 'on');
    linePhi = animatedline(axAtt, 'Color', 'm', 'DisplayName', '\phi');
    lineThe = animatedline(axAtt, 'Color', 'c', 'DisplayName', '\theta');
    title(axAtt, 'Actitud (rad)'); legend(axAtt, 'show');
    xlim(axAtt, [0 T(end)]); ylim(axAtt, [-0.6 0.6]);

    % ==========================================
    % CONFIGURACIÓN DEL VIDEO
    % ==========================================
    if save_video
        % Crea el archivo MP4 en la carpeta actual
        v = VideoWriter('Simulacion_UAV_Chimenea.mp4', 'MPEG-4');
        v.FrameRate = 30; % Fotogramas por segundo (Fluidez)
        v.Quality = 100;  % Calidad máxima
        open(v);
        disp('Iniciando grabación de video... Por favor, no minimices la figura.');
    end

    % --- Bucle de Animación ---
    l = params.l;
    skip = 3; % Aumenta este número si tu PC se pone muy lenta al grabar

    for k = 1:skip:length(T)
        if ~ishandle(hFig), break; end % Salir si se cierra la ventana

        % Estados y Errores
        phi = X(1,k); theta = X(3,k); psi = X(5,k);
        pos = [X(9,k); X(11,k); X(7,k)];
        error_curr = pos - [Traj(1,k); Traj(2,k); Traj(3,k)];

        % Actualizar Telemetría
        addpoints(lineX, T(k), pos(1)); addpoints(lineY, T(k), pos(2)); addpoints(lineZ, T(k), pos(3));
        addpoints(errX, T(k), error_curr(1)); addpoints(errY, T(k), error_curr(2)); addpoints(errZ, T(k), error_curr(3));
        addpoints(linePhi, T(k), phi); addpoints(lineThe, T(k), theta);

        % Rotación del esqueleto
        R = eul2rotm([psi, theta, phi], 'ZYX');
        B_f = pos + R * [l; 0; 0]; B_b = pos + R * [-l; 0; 0];
        B_r = pos + R * [0; l; 0]; B_l = pos + R * [0; -l; 0];

        set(arm_x, 'XData', [B_b(1) B_f(1)], 'YData', [B_b(2) B_f(2)], 'ZData', [B_b(3) B_f(3)]);
        set(arm_y, 'XData', [B_l(1) B_r(1)], 'YData', [B_l(2) B_r(2)], 'ZData', [B_l(3) B_r(3)]);
        set(rotor_f, 'XData', B_f(1), 'YData', B_f(2), 'ZData', B_f(3));
        set(rotors, 'XData', [B_b(1) B_r(1) B_l(1)], 'YData', [B_b(2) B_r(2) B_l(2)], 'ZData', [B_b(3) B_r(3) B_l(3)]);
        addpoints(rastro, pos(1), pos(2), pos(3));

        drawnow limitrate;

        % ==========================================
        % CAPTURA DE FOTOGRAMAS PARA EL VIDEO
        % ==========================================
        if save_video
            frame = getframe(hFig); % Toma una "foto" de toda la figura
            writeVideo(v, frame);   % Añade la foto al archivo de video
        end
    end

    % Cerrar y guardar el video al terminar
    if save_video
        close(v);
        disp('Video guardado con éxito: "Simulacion_UAV_Chimenea.mp4"');
    end
end