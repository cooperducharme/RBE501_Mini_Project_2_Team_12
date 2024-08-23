function V_b = twistspace2body(V_s,T)
    %Given a space frame {s} and a moving body frame {b}, calculate the spatial velocity (twist) V_b as a 6x1 vector in the body frame
    %Inputs: V_s is the spatial velocity as a 6x1 vector in the space frame. T is the homogenious trans. matric between {s} and {b}
    %Output: V_b V_s is the spatial velocity as a 6x1 vector in the body frame.
    
    %extracting rotational and translational components from V_s
    w_s = [V_s(1); V_s(2); V_s(3)];
    v_s = [V_s(4); V_s(5); V_s(6)];
    
    %creating skew sym. matrix of w_s
    w_s_skew = skew(w_s);
    
    %creating skew matrixs of V_s
    V_s_brac = [ w_s_skew v_s; 0 0 0 0]; 
    
    %calculating skew matrics of V_b
    V_b_brac = inv(T)*V_s_brac*T; 
    
    %extracting rotational and translational components from V_b
    w_b_skew = V_b_brac(1:3, 1:3);
    
    w_b = [w_b_skew(3,2);w_b_skew(1,3);w_b_skew(2,1)];
    
    v_b = V_b_brac(1:3, 4);
    
    %assembling V_b as a 6x1 vector
    V_b = [w_b; v_b];
   
    
    
end