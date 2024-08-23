function T = twist2ht(S,theta)

% twist2ht(S,theta)
% This function converts a twist to a homogeneous transformation matrix
% Given S is a screw axis and theta is a scalar of a twist
% This function outputs the corresponding homogeneous transformation
% matrix T

omega = S(1:3); 
v = S(4:6); 

% Function to calculate a rotation matrix: R = axisangle2rot(omega,theta);
    
omega_skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
p = (eye(3)*theta + (1-cos(theta))*omega_skew + (theta-sin(theta))*omega_skew^2)*v;
        
R1 = axisangle2rot(omega,theta);
        
T = [R1 p; 0 0 0 1];
end