function R = axisangle2rot(omega,theta)
% Given the function input of exponential coordinates of a rotation omega theta
% where theta is a scalar
% and omega is 3-dimensions, with magnitude of det(omega) = 1
% 
% This function calculates the corresponding rotation matrix R

omega_skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
R = eye(3) + sin(theta)*omega_skew + (1-cos(theta))*omega_skew^2;
end