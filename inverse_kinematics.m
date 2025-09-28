function [theta1, theta2] = inverse_kinematics(x, y, L1, L2)
% INVERSE_KINEMATICS Calculate joint angles for desired end-effector position
% Inputs:
%   x, y - Desired end-effector position
%   L1, L2 - Link lengths
% Outputs:
%   theta1, theta2 - Joint angles (radians)

    % Calculate intermediate values
    c2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
    
    % Check if position is reachable
    if abs(c2) > 1
        error('Target position (%.2f, %.2f) is unreachable with links L1=%.2f, L2=%.2f', x, y, L1, L2);
    end
    
    % Choose elbow-up configuration
    s2 = sqrt(1 - c2^2);
    theta2 = atan2(s2, c2);
    
    % Calculate theta1
    theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));
end