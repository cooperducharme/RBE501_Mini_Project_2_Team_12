function adV = ad(V)
    % Given a twist V = [w;v]
    % This function calculates the corresponding 6x6 adV matrix
    % This matrix can be used to calculate the Lie Bracet of V1 and another
    % twist V2 as: adV*V2
    w = V(1:3);
    v = V(4:6); 
    w_skew = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
    v_skew = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    
    adV = [w_skew zeros(3); v_skew w_skew];
    
end