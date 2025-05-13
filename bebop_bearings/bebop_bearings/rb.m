clear; clc; close all;

% Parámetros
dt = 0.05; T = 20; N = T/dt;
k_bearing = 1.5;

% Trajectory speed for the leader
trajectory_speed = 0.3;
max_dist_x = 10;

% Inicialización
leader = zeros(2, N);
follower1 = zeros(2, N);
follower2 = zeros(2, N);

% Posiciones iniciales
leader(:,1) = [0; 0];
follower1(:,1) = [-1; -1];
follower2(:,1) = [-1; 1];

% Desired bearings: seguidores mantendrán posición relativa respecto al líder
bearing1_desired = [1; 1]/norm([1; 1]);
bearing2_desired = [1; -1]/norm([1; -1]);

for k = 1:N-1
    t = (k-1)*dt;
    
    % === Trayectoria del líder ===
    x_d = min(t * trajectory_speed, max_dist_x);
    y_d = 0;
    leader(:,k+1) = [x_d; y_d];

    % === Control por bearings para follower 1 ===
    rel_vec1 = leader(:,k) - follower1(:,k);
    bearing1 = rel_vec1 / norm(rel_vec1);
    u1 = k_bearing * (bearing1_desired - bearing1);
    follower1(:,k+1) = follower1(:,k) + u1 * dt;

    % === Control por bearings para follower 2 ===
    rel_vec2 = leader(:,k) - follower2(:,k);
    bearing2 = rel_vec2 / norm(rel_vec2);
    u2 = k_bearing * (bearing2_desired - bearing2);
    follower2(:,k+1) = follower2(:,k) + u2 * dt;
end

% === Visualización ===
figure;
plot(leader(1,:), leader(2,:), 'r-', 'LineWidth', 2); hold on;
plot(follower1(1,:), follower1(2,:), 'b--', 'LineWidth', 1.5);
plot(follower2(1,:), follower2(2,:), 'g--', 'LineWidth', 1.5);
legend('Líder', 'Seguidor 1', 'Seguidor 2');
xlabel('X'); ylabel('Y'); axis equal; grid on;
title('Simulación de formación con control por bearings');
