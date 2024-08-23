function J_b = jacobe(S,M,q) 
    %This function `jacobe' calculates the Jacobian matrix, expressed in the end effector frame, for a robotic arm.
    %The function takes as input a matrix S containing all the screw axes (arranged by column) and a vector of joint variables q. 
    %The screw axes are expressed in the space frame.
    %The function can calculate the Jacobian for kinematic chains with any number `n' of joints.

    J_b = zeros(6, size(S, 2)); % Initialize the Jacobian matrix
    
    % Calculate the space Jacobian
    J_space = jacob0(S, q);
    
    %calc transformation matrix in space frame
    T = fkine(S,M,q,'space');
    
    %calc body Jacobian
    for i = 1:length(q)
        J_b(:,i) = adjoint(J_space(:,i),inv(T));
    end
end