function omega_b = angvelocityspace2body(omega_s,R)
    % Given a space frame {s} and a rotating body frame {b} 
    %calculate the angular velocity, omega_b, in the body frame
    %INPUTs: omega_s is the angular velocity in the space frame. R is the
    %rotation matrix between {s} and {b}
    %Output: omega_b is the angular velocity in the body frame
    omega_b = transpose(R)*omega_s;
end