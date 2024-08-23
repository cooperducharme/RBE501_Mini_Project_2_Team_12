function J = jacob0(S,q) 
% This function calculates the space Jacobian
% jacob0 = (S,q)
% Given S is a matrix contating all of the screw axes (arranged by column)
% and  vector of joint variables, q 
% This function can calculate the space Jacobian for kinematic chains with
% any number 'n; of joints


    J = zeros(6,size(S,2));
    J(:,1) = S(:,1);

    for i =2:size(S,2)
        Tj= eye(4);
        for n = 1:size(S,2)
            Tj = Tj*twist2ht(S(:,n), q(n));
            J(:,n)=adjoint(S(:,n),Tj);
        end
    end
   end