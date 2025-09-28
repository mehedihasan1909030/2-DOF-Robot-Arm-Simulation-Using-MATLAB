function [x, y] = forward_kinematics(theta1, theta2, L1, L2)
% FORWARD_KINEMATICS Calculate end-effector position from joint angles
% Inputs:
%   theta1 - First joint angle (radians)
%   theta2 - Second joint angle (radians)
%   L1 - Length of first link (meters)
%   L2 - Length of second link (meters)
% Outputs:
%   x, y - End-effector position coordinates

    x = L1*cos(theta1) + L2*cos(theta1 + theta2);
    y = L1*sin(theta1) + L2*sin(theta1 + theta2);
end