function twist_inB = adjoint(twist_inA,T_AB)
% Adjoint Transformations
% Given the inputs of: 
% A twist with respect to some arbitrary reference frame,twist_A
% And the homogeneous transformation matrix which describes the position
% and orientation of a new frame, T_AB.
%
% This function calculates the twist representation in this new frame,
% twist_inB
    R = T_AB(1:3,1:3);
    p = T_AB(1:3,4);
    p_skew = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    
    twist_inB = [R zeros(3); p_skew*R R]*twist_inA;
end

