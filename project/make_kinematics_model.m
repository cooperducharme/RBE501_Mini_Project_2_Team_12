function [S,M] = make_kinematics_model()
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% the BARRET WAM 7DOF robot.
%
% Inputs: None
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

% Implement Kinematic Model
% Link Lengths in (m)
L1 = 0.550; 
L2 = 0.3;
L3 = 0.06;
W1 = 0.045;
  
% Home Configuration and Twists
R = eye(3);
p = [0, 0, L1+L2+L3]';
M = [R p; zeros(1,3) 1];

B = [0, 0, 1,     0,    0, 0;
     0, 1, 0, L1+L2+L3, 0, 0;
     0, 0, 1,     0,    0, 0;
     0, 1, 0,   L2+L3,  0, W1;
     0, 0, 1,     0,    0, 0;
     0, 1, 0,     L3,   0, 0;
     0, 0, 1,     0,    0, 0]';

S = zeros(6, 7);
for i=1:7
    S(:, i) = Adjoint2(M)*(B(:, i));
end   


end