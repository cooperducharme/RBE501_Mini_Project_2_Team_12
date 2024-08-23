function T = fkine(S,M,q,frame)

    % fkine(S,M,q,frame) calculates th forward kinematics of a robotics arm using the
    % Product of Exponentials Formula.
    %
    % The function takes as an input S containing all of the screw axes
    % (arranged by column), a vector joint variable q, and a homogeneous
    % transformation matrix M representing the robot pose in its home
    % configuration, and frame which can be an input of either 'space' or
    % 'body' to controls whether the product of exponentials is calculated 
    % in the space or body frame of the robot.
    
    if length(frame) == 5

        T= eye(4);
       
        for i = 1:size(S,2)
            T = T*twist2ht(S(:,i), q(i));
        end
        T = T*M;
    else
        %body frame calc
        T= eye(4);
       
        for i = 1:size(S,2)
            T = T*twist2ht(S(:,i), q(i));
        end
        T = M*T;
    end
end