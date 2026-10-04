% Simulación B: SMC Ideal
clc; clear; close all;

addpath('../controllers');
addpath('../models');
addpath('../utils');
% ------------------------------------------------

% Carga de parámetros idéntica al archivo main_basic
params.m = 1.3; params.l = 0.224; params.Ix = 1.214e-2; params.Iy = 1.214e-2; params.Iz = 2.061e-2;
params.g = 9.81; params.a1 = (params.Iy - params.Iz)/params.Ix; params.a2 = (params.Iz - params.Ix)/params.Iy;
params.a3 = (params.Ix - params.Iy)/params.Iz; params.b1 = params.l/params.Ix; params.b2 = params.l/params.Iy; params.b3 = params.l/params.Iz;

% Configuración de simulación
dt = 0.005; tf = 20; T = 0:dt:tf; N = length(T);
X = zeros(12, N); 
X(9, 1) = -3; % Posición inicial X en la galería inferior
U_log = zeros(4, N-1); Traj_log = zeros(3, N);
dist = struct('x', 0, 'y', 0, 'z', 0); % Ideal

for k = 1:N-1
    t = T(k); x_curr = X(:,k);

    % Generador de trayectoria
    traj = trajectory_generator(t);
    Traj_log(:,k) = [traj.x; traj.y; traj.z];

    % Controlador SMC Robust
    u = smc_controller(x_curr, traj, params);
    U_log(:,k) = u;

    dx = drone_dynamics(t, x_curr, u, params, dist);
    X(:,k+1) = x_curr + dx * dt;
end
Traj_log(:,N) = [traj.x; traj.y; traj.z];

plot_results(T, X, U_log, Traj_log, 'Control SMC (Ideal / Sin Perturbaciones)');