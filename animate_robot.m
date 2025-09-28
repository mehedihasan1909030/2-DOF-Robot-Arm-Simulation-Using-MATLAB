function animate_robot(theta1_traj, theta2_traj, L1, L2, x_desired, y_desired)
% ANIMATE_ROBOT Animate the robot arm following a trajectory
% Inputs:
%   theta1_traj, theta2_traj - Joint angle trajectories
%   L1, L2 - Link lengths
%   x_desired, y_desired - Desired end-effector path (optional)

    figure('Position', [100, 100, 800, 600]);
    
    % Store trajectory for plotting
    x_actual = zeros(size(theta1_traj));
    y_actual = zeros(size(theta1_traj));
    
    for i = 1:length(theta1_traj)
        clf;
        
        % Calculate current positions
        theta1 = theta1_traj(i);
        theta2 = theta2_traj(i);
        
        % Joint positions
        x0 = 0; y0 = 0;  % Base
        x1 = L1*cos(theta1);
        y1 = L1*sin(theta1);
        x2 = x1 + L2*cos(theta1 + theta2);
        y2 = y1 + L2*sin(theta1 + theta2);
        
        % Store actual end-effector position
        x_actual(i) = x2;
        y_actual(i) = y2;
        
        % Plot robot links
        plot([x0 x1 x2], [y0 y1 y2], 'b-', 'LineWidth', 4);
        hold on;
        
        % Plot joints
        plot([x0 x1 x2], [y0 y1 y2], 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'black');
        
        % Highlight end-effector
        plot(x2, y2, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'red');
        
        % Plot desired path if provided
        if nargin > 4
            plot(x_desired, y_desired, 'g--', 'LineWidth', 2);
        end
        
        % Plot actual trajectory so far
        if i > 1
            plot(x_actual(1:i), y_actual(1:i), 'r-', 'LineWidth', 2);
        end
        
        % Plot workspace boundaries
        theta_circle = 0:0.1:2*pi;
        plot((L1+L2)*cos(theta_circle), (L1+L2)*sin(theta_circle), 'k:', 'LineWidth', 1);
        plot(abs(L1-L2)*cos(theta_circle), abs(L1-L2)*sin(theta_circle), 'k:', 'LineWidth', 1);
        
        % Formatting
        axis equal;
        grid on;
        xlim([-2.2, 2.2]);
        ylim([-2.2, 2.2]);
        title(sprintf('2-DOF Robot Arm - Step %d/%d', i, length(theta1_traj)));
        xlabel('X (m)');
        ylabel('Y (m)');
        
        if nargin > 4
            legend('Robot Arm', 'Joints', 'End-Effector', 'Desired Path', 'Actual Path', 'Workspace', 'Location', 'best');
        else
            legend('Robot Arm', 'Joints', 'End-Effector', 'Actual Path', 'Workspace', 'Location', 'best');
        end
        
        pause(0.1);
    end
    
    fprintf('Animation complete!\n');
end