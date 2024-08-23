function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%

% Forward iterations
g = params.g;
S = params.S; %screw axis in space frame
M = params.M; 
G = params.G;
Ftip = params.Ftip; %wrench at end-effector

jointPos = params.jointPos;
jointVel = params.jointVel;
jointAcc = params.jointAcc;

%number of joints in the robot
n = size(params.jointPos,1);
V = zeros(6,n+1);
Vdot = zeros(6,n+1);
Vdot(4:6,1) = -g; %gravity accel acting on space frame

Ai = zeros(6,n); %screw axes expressed in local link frame
T = zeros(4,4,n); %homogen. transformation matrix
Mi = eye(4); %home config. matrix
AdTi = zeros(6,6,n+1); %adjoint matrix vectors
AdTi(:,:,n+1) = Adjoint2(pinv(M(:,:,n+1)));

for ii = 1:n
    Mi = Mi *M(:,:,ii);
    
    Ai(:,ii) = Adjoint2(pinv(Mi))*S(:,ii); % current screw axes w.r.t local link frame
    
    T(:,:,ii) = twist2ht(Ai(:,ii),-jointPos(ii))* pinv(M(:,:,ii)); % Eq (8.50)
    
    AdTi(:,:,ii) = Adjoint2(T(:,:,ii));
    
    %calc the link velocities
    V(:,ii+1) = AdTi(:,:,ii)* V(:,ii) + Ai(:,ii) * jointVel(ii); %E1 (8.51)
    %note that V(:,1) is the space frame velocity
    
    %calc the link acceleration
    Vdot(:,ii+1) = AdTi(:,:,ii)*Vdot(:,ii) + ad(V(:,ii+1))*Ai(:,ii) * jointVel(ii) + Ai(:,ii) * jointAcc(ii); %Eq (8.52)
end



% Backward iterations
tau = zeros(n,1);

Fi = Ftip;

for ii = n:-1:1
    Fi = AdTi(:,:,ii+1)'*Fi + G(:,:,ii)*Vdot(:,ii+1) - ad(V(:,ii+1))'*(G(:,:,ii)*V(:,ii+1));%Eq (8.53)
    
    tau(ii) = Fi' * Ai(:,ii);%Eq (8.54)
end


end