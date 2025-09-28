%% 2-DOF Robot Arm Simulation
% This script demonstrates forward/inverse kinematics and trajectory following

clear; clc; close all;

%% Robot Parameters
L1 = 1.0;  % First link length (m)
L2 = 0.8;  % Second link length (m)

fprintf('Robot Parameters:\n');
fprintf('L1 = %.2f m\n', L1);
fprintf('L2 = %.2f m\n', L2);
fprintf('Workspace: Inner radius = %.2f m, Outer radius = %.2f m\n\n', abs(L1-L2), L1+L2);

%% Test Forward Kinematics
fprintf('=== Testing Forward Kinematics ===\n');
test_theta1 = pi/4;
test_theta2 = pi/6;
[test_x, test_y] = forward_kinematics(test_theta1, test_theta2, L1, L2);
fprintf('Joint angles: θ1=%.2f°, θ2=%.2f°\n', test_theta1*180/pi, test_theta2*180/pi);
fprintf('End-effector position: (%.3f, %.3f)\n\n', test_x, test_y);

%% Test Inverse Kinematics
fprintf('=== Testing Inverse Kinematics ===\n');
try
    [calc_theta1, calc_theta2] = inverse_kinematics(test_x, test_y, L1, L2);
    fprintf('Calculated joint angles: θ1=%.2f°, θ2=%.2f°\n', calc_theta1*180/pi, calc_theta2*180/pi);
    fprintf('Error: θ1=%.4f°, θ2=%.4f°\n\n', (test_theta1-calc_theta1)*180/pi, (test_theta2-calc_theta2)*180/pi);
catch ME
    fprintf('Error: %s\n\n', ME.message);
end

%% Define Trajectory - Circular Path
fprintf('=== Generating Circular Trajectory ===\n');
t = 0:0.05:2*pi;  % Time/parameter vector
radius = 0.5;
center_x = 1.2;
center_y = 0.3;

x_desired = center_x + radius*cos(t);
y_desired = center_y + radius*sin(t);

fprintf('Trajectory: Circle with radius %.2f m, center (%.2f, %.2f)\n', radius, center_x, center_y);
fprintf('Number of points: %d\n\n', length(t));

%% Calculate Joint Trajectories
fprintf('=== Computing Joint Trajectories ===\n');
theta1_traj = zeros(size(t));
theta2_traj = zeros(size(t));

successful_points = 0;
for i = 1:length(t)
    try
        [theta1_traj(i), theta2_traj(i)] = inverse_kinematics(x_desired(i), y_desired(i), L1, L2);
        successful_points = successful_points + 1;
    catch ME
        fprintf('Warning: Point %d unreachable - %s\n', i, ME.message);
        if i > 1
            % Use previous values
            theta1_traj(i) = theta1_traj(i-1);
            theta2_traj(i) = theta2_traj(i-1);
        end
    end
end

fprintf('Successfully calculated %d/%d points\n\n', successful_points, length(t));

%% Animate the Robot
fprintf('=== Starting Animation ===\n');
fprintf('Close the figure window to continue...\n');
animate_robot(theta1_traj, theta2_traj, L1, L2, x_desired, y_desired);

%% Plot Joint Angles Over Time
figure('Position', [200, 200, 800, 600]);
subplot(2,1,1);
plot(t, theta1_traj*180/pi, 'b-', 'LineWidth', 2);
title('Joint 1 Angle vs Time');
ylabel('Angle (degrees)');
grid on;
ylim([-180, 180]);

subplot(2,1,2);
plot(t, theta2_traj*180/pi, 'r-', 'LineWidth', 2);
title('Joint 2 Angle vs Time');
xlabel('Time Parameter');
ylabel('Angle (degrees)');
grid on;
ylim([-180, 180]);

%% Plot End-Effector Path Comparison
figure('Position', [300, 300, 800, 600]);
x_actual = zeros(size(t));
y_actual = zeros(size(t));

for i = 1:length(t)
    [x_actual(i), y_actual(i)] = forward_kinematics(theta1_traj(i), theta2_traj(i), L1, L2);
end

plot(x_desired, y_desired, 'g--', 'LineWidth', 3, 'DisplayName', 'Desired Path');
hold on;
plot(x_actual, y_actual, 'r-', 'LineWidth', 2, 'DisplayName', 'Actual Path');

% Plot workspace
theta_circle = 0:0.1:2*pi;
plot((L1+L2)*cos(theta_circle), (L1+L2)*sin(theta_circle), 'k:', 'LineWidth', 1, 'DisplayName', 'Outer Workspace');
plot(abs(L1-L2)*cos(theta_circle), abs(L1-L2)*sin(theta_circle), 'k:', 'LineWidth', 1, 'DisplayName', 'Inner Workspace');

axis equal;
grid on;
title('End-Effector Path Comparison');
xlabel('X (m)');
ylabel('Y (m)');
legend('show');

%% Calculate and Display Errors
path_error = sqrt((x_desired - x_actual).^2 + (y_desired - y_actual).^2);
fprintf('=== Path Following Analysis ===\n');
fprintf('Maximum path error: %.4f m\n', max(path_error));
fprintf('Average path error: %.4f m\n', mean(path_error));
fprintf('RMS path error: %.4f m\n', sqrt(mean(path_error.^2)));

fprintf('\nSimulation complete!\n');