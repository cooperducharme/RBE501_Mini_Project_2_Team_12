clc; clear; close all;
addpath('../lib');
addpath('../mr');

%Gravity Compensation

%% Create the 7 DOF WAM manipulator
% DH parameters
a = [0, 0, 0.045, -0.045, 0, 0, 0];
alpha = [-pi/2, pi/2, -pi/2, pi/2, -pi/2, pi/2, 0];
d = [0, 0, 0.55, 0, 0.3, 0, 0.06];
theta_offset = [0, 0, 0, 0, 0, 0, 0];  % Replace these with actual joint offsets if they are not zero

% Create each Link using the DH parameters
links = Link.empty(0,7);
for i = 1:7
    links(i) = Revolute('d', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', theta_offset(i));
end

% Define the robot using the SerialLink class
robot = SerialLink(links, 'name', 'WAM 7DOF');

% Define joint limits (example limits, replace with actual limits)
qlim = [-2.6  2.6;    % q1
        -2  2;    % q2
        -2.8  2.8;    % q3
        -0.9  3.1;    % q4
        -4.76  1.24;    % q5
        -1.6  1.6;    % q6
        -3  3];   % q7
for i = 1:7
    robot.links(i).qlim = qlim(i,:);
end

% Display the robot
q = zeros(1,7);  % Home position
robot.plot(q);

%% Kinematics and Dynamics Inputs

% Create a kinematic model of the robot
[S,M] = make_kinematics_model();
n = size(S,2); % read the number of joints

% Create a dynamical model of the robot %Mlist includes end-eff frame
[Mlist,Glist] = make_dynamics_model();

%% Gravity compensation
fprintf('-----------------------Gravity Compensation-----------------------\n');
% We are now going to solve the inverse dynamics and calculate the torques
% required to keep the robot where it is.
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Initialize the parameters for the RNE algorithm
clear params

params.g = g; % gravity vector
params.S = S; % screw axes
params.M = Mlist; % link frames 
params.G = Glist; % inertial properties
params.jointPos = zeros(n,1); % desired joint positions
params.jointVel = zeros(n,1); % desired joint velocities
params.jointAcc = zeros(n,1); % desired joint accelerations
params.Ftip = zeros(6,1);     % desired wrench at the end effector

% Invoke the RNE algorithm to calculate the joint torques needed for
% gravity compensation
tau = rne(params);

fprintf('Joint Torques: ');
fprintf('[%f %f %f %f %f %f %f] Nm\n', tau(1), tau(2), tau(3), tau(4), tau(5), tau(6), tau(7));

% To make sure that the solution is correct, let us simulate the robot
fprintf('\nWe are now going to simulate the robot to see if it moves.\n');
fprintf('Calculating the Forward Dynamics: ');
nbytes = fprintf('0%%');

dt = 1e-4;        % simulation time step [s]
t = 0 : dt : 0.1; % total simulation time [s]

qt = zeros(n,size(t,2));  qt(:,1) = params.jointPos;
qdt = zeros(n,size(t,2)); qdt(:,1) = params.jointVel;

% Display the robot
robot.plot(params.jointPos');

for ii = 1 : size(t,2) - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(ii/(size(t,2)-1)));

    % Set torques
    params.tau = tau; % supply the torques needed for gravity compensation

    % Calculate the joint accelerations
    jointAcc = fdyn(params);

    % Integrate the joint accelerations to get velocity and
    % position
    params.jointPos = params.jointPos + dt * params.jointVel;
    params.jointVel = params.jointVel + dt * jointAcc;

    % Accumulate results
    qt(:,ii+1) = params.jointPos;
    qdt(:,ii+1) = params.jointVel;

end

% torques, positions, vels, and accels needed for plotting
t_acc = t;
tau_acc = zeros(n,length(t));
tau_acc(1,:) = tau(1);
tau_acc(2,:) = tau(2);
tau_acc(3,:) = tau(3);
tau_acc(4,:) = tau(4);
tau_acc(5,:) = tau(5);
tau_acc(6,:) = tau(6);
tau_acc(7,:) = tau(7);
jointPos_acc = zeros(n,length(t));
jointVel_acc = zeros(n,length(t));
jointAcc_acc = zeros(n,length(t));

fprintf('\nDone. Simulating the robot...\n');
title('Gravity Compensation');
robot.plot(qt(:,1:100:end)');

%% plot Joint Positions
figure;
hold on, grid on
plot(t_acc, jointPos_acc(1,:), 'Linewidth', 2);
plot(t_acc, jointPos_acc(2,:), 'Linewidth', 2);
plot(t_acc, jointPos_acc(3,:), 'Linewidth', 2);
plot(t_acc, jointPos_acc(4,:), 'Linewidth', 2);
plot(t_acc, jointPos_acc(5,:), 'Linewidth', 2);
plot(t_acc, jointPos_acc(6,:), 'Linewidth', 2);
plot(t_acc, jointPos_acc(7,:), 'Linewidth', 2);
hold off;
title('Joint Positions');
xlabel('Time [s]'), ylabel('Position [m]');
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'});
set(gca, 'FontSize', 9);

%% Plot Joint Velocity
figure;
hold on, grid on
plot(t_acc, jointVel_acc(1,:), 'Linewidth', 2);
plot(t_acc, jointVel_acc(2,:), 'Linewidth', 2);
plot(t_acc, jointVel_acc(3,:), 'Linewidth', 2);
plot(t_acc, jointVel_acc(4,:), 'Linewidth', 2);
plot(t_acc, jointVel_acc(5,:), 'Linewidth', 2);
plot(t_acc, jointVel_acc(6,:), 'Linewidth', 2);
plot(t_acc, jointVel_acc(7,:), 'Linewidth', 2);
hold off;
title('Joint Velocity');
xlabel('Time [s]'), ylabel('Velocity [m/s]');
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'});
set(gca, 'FontSize', 9);

%% Plot Joint Accelerations
figure;
hold on, grid on
plot(t_acc, jointAcc_acc(1,:), 'Linewidth', 2);
plot(t_acc, jointAcc_acc(2,:), 'Linewidth', 2);
plot(t_acc, jointAcc_acc(3,:), 'Linewidth', 2);
plot(t_acc, jointAcc_acc(4,:), 'Linewidth', 2);
plot(t_acc, jointAcc_acc(5,:), 'Linewidth', 2);
plot(t_acc, jointAcc_acc(6,:), 'Linewidth', 2);
plot(t_acc, jointAcc_acc(7,:), 'Linewidth', 2);
hold off;
title('Joint Acceleration');
xlabel('Time [s]'), ylabel('Acceleration [m/s^2]');
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'});
set(gca, 'FontSize', 9);

%% Plot Joint Torques
figure, hold on, grid on
plot(t_acc, tau_acc(1,:), 'Linewidth', 2);
plot(t_acc, tau_acc(2,:), 'Linewidth', 2);
plot(t_acc, tau_acc(3,:), 'Linewidth', 2);
plot(t_acc, tau_acc(4,:), 'Linewidth', 2);
plot(t_acc, tau_acc(5,:), 'Linewidth', 2);
plot(t_acc, tau_acc(6,:), 'Linewidth', 2);
plot(t_acc, tau_acc(7,:), 'Linewidth', 2);
title('Torque Profiles');
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'});
set(gca, 'FontSize', 9);

input('Simulation complete. Press Enter to continue.');