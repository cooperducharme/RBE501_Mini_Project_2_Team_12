function traj = make_trajectory(type, params)
% This function generates either cubic or quintic polynomial trajectories
%
% INPUT:
% % type - a string indicating the type of trajectory that you want to generate.
%                Acceptable values: {'cubic' | 'quintic'}
% 
% params - a structure containing the prescribed trajectory parameters
%   params.t  - 2-vector prescribing the initial and final time
%   params.dt - desired time step
%   params.q - 2-vector prescribing the starting and final positions
%   params.v - 2-vector describing the starting and final velocities
%   params.a - 2-vector describing the starting and final accelerations (only for quintic polynomials)
%
% OUTPUT:
% traj - a structure containing the trajectory
%   traj.t - n-vector representing time
%   traj.q - n-vector representing position over time
%   traj.v - n-vector representing velocity over time
%   traj.a - n-vector representing acceleration over time
 traj.t = params.t(1):params.dt:params.t(2);

    if strcmp(type,'cubic')
        %Coefficients
        a = zeros(4,1);
        q_m = [params.q(1) params.v(1) params.q(2) params.v(2)]';
        t_m = [1 params.t(1) params.t(1)^2 params.t(1)^3;
               0 1 2*params.t(1) 3*params.t(1)^2;
               1 params.t(2) params.t(2)^2 params.t(2)^3;
               0 1 2*params.t(2) 3*params.t(2)^2];
        a = t_m\q_m;
        a0 = a(1);
        a1 = a(2);
        a2 = a(3);
        a3 = a(4);

        % Position, velocity, and acceleration
        traj.q = a0 + a1*traj.t + a2*traj.t.^2 + a3*traj.t.^3;
        traj.v = a1 + 2*a2*traj.t + 3*a3*traj.t.^2;
        traj.a = 2*a2 + 6*a3*traj.t;

    elseif strcmp(type, 'quintic')
        % Quintic polynomial coefficients
        a_m = zeros(6,1);
        q_m = [params.q(1) params.v(1) params.a(1) params.q(2) params.v(2) params.a(2)]';
        t_m = [1 params.t(1) params.t(1)^2 params.t(1)^3 params.t(1)^4 params.t(1)^5;
               0 1 2*params.t(1) 3*params.t(1)^2 4*params.t(1)^3 5*params.t(1)^4;
               0 0 2 6*params.t(1) 12*params.t(1)^2 20*params.t(1)^3;
               1 params.t(2) params.t(2)^2 params.t(2)^3 params.t(2)^4 params.t(2)^5;
               0 1 2*params.t(2) 3*params.t(2)^2 4*params.t(2)^3 5*params.t(2)^4;
               0 0 2 6*params.t(2) 12*params.t(2)^2 20*params.t(2)^3];
        a_m = t_m\q_m;
        a0 = a_m(1);
        a1 = a_m(2);
        a2 = a_m(3);
        a3 = a_m(4);
        a4 = a_m(5);
        a5 = a_m(6);

        % Position, velocity, and acceleration
        traj.q = a0 + a1*traj.t + a2*traj.t.^2 + a3*traj.t.^3 + a4*traj.t.^4 + a5*traj.t.^5;
        traj.v = a1 + 2 * a2 * traj.t + 3*a3 * traj.t.^2 + 4*a4 * traj.t.^3 + 5*a5 * traj.t.^4;
        traj.a = 2*a2 + 6*a3 * traj.t + 12*a4 * traj.t.^2 + 20 * a5 * traj.t.^3;

    else
        error('Invalid trajectory type');
    end
end
