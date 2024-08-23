function J_a = jacoba(S,M,q)    
    % This function calculates the analytic jacobian matrix, J_a
    % This matrix provides a mapping between the joint velocities  
    % and the end effector Cartesian velocity , expressed in the space frame, 
    % i.e.: p_dot = J_a * q_dot
    %The function takes as input a matrix S containing all the screw axes 
    % (arranged by column, expressed in the space frame), the homogeneous 
    % transformation matrix M representing the home configuration, and 
    % a vector of joint variables q.
 
    %initlizing variable
    J_a = zeros(3,length(q));
    J_s = zeros(6,length(q));
    Jv = zeros(3,length(q));
    Jw = zeros(3,length(q));
    
    %calc homogen trans matrix
    T = fkine(S,M,q,'space');

    %extracting p from T
    p = T(1:3,4);
    
    % calc skew sym matrix of p
    ps = skew(p);
    
    J_s = jacob0(S,q);
    
    %extracting Jv and Jw from J_s
    Jv = J_s(4:6,:);
    Jw = J_s(1:3,:);
    
    %calculating J_a
    J_a = Jv - ps*Jw;
end