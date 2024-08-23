% Inverse Kinematics
% function  ikin(S,M,currentQ,targetPose)
function q  = ikin(S,M,currentQ, targetPose)
    counter = 0; %counter to halt execution
    max_cycles = 3000; %setting max. cycles for ikin

    
    % Generate the robot's pose
%     T = fkine(S,M,targetPose');
%     targetPose = MatrixLog6(T);
%     targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';
%     
  
    % Calculate the twist representing the robot's home pose
    currentPose = MatrixLog6(M);
    currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';
        
    % Inverse Kinematics
    while norm(targetPose - currentPose) > 1e-6 
        J = jacob0(S,currentQ);
        %deltaQ = pinv(J)*(targetPose-currentPose);
        lambda = 0.4; 
        deltaQ = J' * pinv(J*J' + lambda^2 * eye(6)) * (targetPose - currentPose);

%         alpha = 0.1;
%         deltaQ = alpha*J' * (targetPose - currentPose);

      
        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ,'space');
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';

        counter = counter + 1;


        if counter >= max_cycles
            fprintf('\nMax. Cycles Reached for this config')
            q = [0 0 0 0 0 0 0];
            break; %exit the while loop
        end
        q = currentQ;
        
        
    end

end

