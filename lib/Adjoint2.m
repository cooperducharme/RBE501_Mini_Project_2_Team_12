function AdT = Adjoint2(T)
    % Given a homogeneous transformation matrix T of SE(3), this function
    % calculates the corresponding 6x6 adjoint transformation matrix Ad
    R = T(1:3,1:3);
    p = T(1:3,4);
    p_skew = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    
    AdT = [R zeros(3); p_skew*R R];
    
end