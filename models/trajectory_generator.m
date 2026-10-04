function traj = trajectory_generator(t)
    % Planificador de ruta para entorno confinado
    % Fases:
    % 1. (0s - 4s): Despegue y avance al centro de la chimenea.
    % 2. (4s - 12s): Ascenso vertical por el ducto.
    % 3. (12s - 16s): Ingreso a galería superior y aterrizaje.
    % 4. (> 16s): Reposo en el suelo superior.

    % Inicializar estructura
    traj.y = 0; traj.y_dot = 0; traj.y_ddot = 0;
    traj.psi = 0;

    if t < 4
        % Galería inferior: de X=-3 a X=0, elevándose a Z=1
        [p, dp, ddp] = poly5(t, 0, 4);
        traj.x = -3 + 3*p;     traj.x_dot = 3*dp;     traj.x_ddot = 3*ddp;
        traj.z = 0 + 1*p;      traj.z_dot = 1*dp;     traj.z_ddot = 1*ddp;
        
    elseif t < 12
        % Ascenso por chimenea: de Z=1 a Z=10
        [p, dp, ddp] = poly5(t, 4, 12);
        traj.x = 0;            traj.x_dot = 0;        traj.x_ddot = 0;
        traj.z = 1 + 9*p;      traj.z_dot = 9*dp;     traj.z_ddot = 9*ddp;
        
    elseif t < 16
        % Galería superior: de X=0 a X=3, aterrizando en Z=9.5
        [p, dp, ddp] = poly5(t, 12, 16);
        traj.x = 0 + 3*p;      traj.x_dot = 3*dp;     traj.x_ddot = 3*ddp;
        traj.z = 10 - 0.5*p;   traj.z_dot = -0.5*dp;  traj.z_ddot = -0.5*ddp;
        
    else
        % Reposo final
        traj.x = 3;            traj.x_dot = 0;        traj.x_ddot = 0;
        traj.z = 9.5;          traj.z_dot = 0;        traj.z_ddot = 0;
    end
end

% Función auxiliar para generar trayectorias suaves (Minimización de Jerk)
function [p, dp, ddp] = poly5(t, t0, t1)
    dt = t1 - t0;
    s = (t - t0) / dt;
    s = max(0, min(1, s)); % Saturación de seguridad
    
    p = 10*s^3 - 15*s^4 + 6*s^5;
    if s > 0 && s < 1
        dp = (30*s^2 - 60*s^3 + 30*s^4) / dt;
        ddp = (60*s - 180*s^2 + 120*s^3) / (dt^2);
    else
        dp = 0; ddp = 0;
    end
end