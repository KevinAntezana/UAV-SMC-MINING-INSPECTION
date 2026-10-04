% Proyecto: Paper de Control de UAVs en Chimeneas Mineras
clc; clear; close all;

addpath('../controllers');
addpath('../models');
addpath('../utils');
% ------------------------------------------------

params.m = 1.3; params.l = 0.224; params.Ix = 1.214e-2; params.Iy = 1.214e-2; params.Iz = 2.061e-2;
params.g = 9.81; params.a1 = (params.Iy - params.Iz)/params.Ix; params.a2 = (params.Iz - params.Ix)/params.Iy;
params.a3 = (params.Ix - params.Iy)/params.Iz; params.b1 = params.l/params.Ix; params.b2 = params.l/params.Iy; params.b3 = params.l/params.Iz;

% Configuración de simulación
dt = 0.005; tf = 20; T = 0:dt:tf; N = length(T);
X = zeros(12, N); 
X(9, 1) = -3; % Posición inicial X en la galería inferior
U_log = zeros(4, N-1); 
Traj_log = zeros(3, N);
Dist_log = zeros(3, N-1); % Registro de perturbaciones

for k = 1:N-1
    t = T(k);
    x_curr = X(:,k);

    % --- SLAM LiDAR (Sensor Noise Approximation) ---
    x_meas = x_curr;
    x_meas(7)  = x_curr(7)  + 0.04 * randn; % Ruido Z Lidar
    x_meas(9)  = x_curr(9)  + 0.02 * randn; % Ruido X Lidar
    x_meas(11) = x_curr(11) + 0.02 * randn; % Ruido Y Lidar

    % Generador de Trayectoria 
    traj = trajectory_generator(t);
    Traj_log(:,k) = [traj.x; traj.y; traj.z];

    % Dinámica de Perturbaciones (Viento + Interacción con Paredes)
    dist = disturbances(x_curr, t);
    Dist_log(:,k) = [dist.x; dist.y; dist.z]; % Guardar historial

    % Cálculo del controlador SMC con estados ruidosos (x_meas)
    u = smc_controller(x_meas, traj, params);
    U_log(:,k) = u;

    % Actualización de la dinámica usando el estado REAL (x_curr)
    dx = drone_dynamics(t, x_curr, u, params, dist);
    X(:,k+1) = x_curr + dx * dt;
end
traj_final = trajectory_generator(T(N));
Traj_log(:,N) = [traj_final.x; traj_final.y; traj_final.z];

% 1. Gráfica Principal (Trayectoria, Error, Control)
plot_results(T, X, U_log, Traj_log, 'Control SMC Robusto con SLAM Lidar');

% 2. NUEVA GRÁFICA: Perturbaciones vs Tiempo
figure('Name', 'Análisis de Perturbaciones', 'Color', 'w', 'Position', [150, 150, 800, 600]);

subplot(3,1,1);
plot(T(1:end-1), Dist_log(1,:), 'r', 'LineWidth', 1);
grid on; ylabel('Dist. X [m/s^2]');
title('Perturbaciones a lo largo de la inspección');
legend('Viento + Repulsión de Pared (Eje X)', 'Location', 'best');

subplot(3,1,2);
plot(T(1:end-1), Dist_log(2,:), 'g', 'LineWidth', 1);
grid on; ylabel('Dist. Y [m/s^2]');
legend('Viento + Repulsión de Pared (Eje Y)', 'Location', 'best');

subplot(3,1,3);
plot(T(1:end-1), Dist_log(3,:), 'b', 'LineWidth', 1);
grid on; xlabel('Tiempo [s]'); ylabel('Dist. Z [m/s^2]');
legend('Corrientes Térmicas Verticales (Eje Z)', 'Location', 'best');

animate_uav(T, X, Traj_log, params);