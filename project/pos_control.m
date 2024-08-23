clc; clear; close all;
addpath('../lib');
addpath('../mr');
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


%% Control the motion of the robot between 2 set points
fprintf('----------------------Dynamic Control of a 3-DoF Arm--------------------\n');

% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

path = [0 0 0.91; 0.5 0.5 0.4]'; %Path is home config to 1 test point %where [0 0 0.91] is the home config

rpy = [1.0501 0.8121 -0.789]; %roll,pitch,yaw values for test point

nPts = size(path,2);

fprintf('Calculating the Inverse Kinematics... ');
robot.plot(zeros(1,n)); hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
title('Inverse Dynamics Control');

% Calculate the inverse kinematics
waypoints = zeros(n,nPts);

for ii = 2:nPts %change 1 to 2 when starting from the home configuration

    R = eul2rotm(rpy);
    p = path(:,ii);

    T = [R p; 0 0 0 1]; %creating T with target xyz and rpy
    targetPose = MatrixLog6(T);
    targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';

    q0 = zeros(1,n);
    
    waypoints(:,ii) = ikin(S,M,q0,targetPose);
    %waypoints(:,ii) = robot.ikine(T);
end

% Solving inverse kinematics with only cartesian points, no roll,pitch,yaw
% for ii = 2:nPts %change 1 to 2 when starting from the home configuration
% 
%     q0 = zeros(1,n);
%     waypoints(:,ii) = ikin(S,M,q0,path(:,ii));
% end
% 
% Calculating rpy from solution of this ikin
% T = fkine(S,M,waypoints(:,2),'space');
% rpy = tr2eul(T) 

fprintf('Done.\n');

% Now, for each pair of consecutive waypoints, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generating the Trajectory and Torque Profiles... ');
nbytes = fprintf('0%%');

% Inititalize the variables where we will store the torque profiles, joint
% positions, and time, so that we can display them later
tau_acc = [];
jointPos_acc = [];
t_acc = [];
jointVel_acc = [];
jointAcc_acc = [];


for jj = 1 : nPts - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(jj/(nPts - 1)));
   
    % Initialize the time vector
    dt = 1e-3;       % time step [s]
    t  = 0 : dt : 0.5; % total time [s]

    % Initialize the arrays where we will accumulate the output of the robot
    % dynamics
    jointPos_prescribed = zeros(n,size(t,2)); % Joint Variables (Prescribed)
    jointVel_prescribed = zeros(n,size(t,2)); % Joint Velocities (Prescribed)
    jointAcc_prescribed = zeros(n,size(t,2)); % Joint Accelerations (Prescribed)
    tau_prescribed      = zeros(n,size(t,2)); % Joint Torques

    jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
    jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)

    % For each joint
    for ii = 1 : n
        % Calculate a trajectory using a quintic polynomial
        params_traj.t = [0 t(end)]; % start and end time of each movement step
        params_traj.dt = dt;
        params_traj.q = [waypoints(ii,jj) waypoints(ii,jj+1)];
        params_traj.v = [0 0];
        params_traj.a = [0 0];

        traj = make_trajectory('quintic', params_traj);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        jointPos_prescribed(ii,:) = traj.q;
        jointVel_prescribed(ii,:) = traj.v;
        jointAcc_prescribed(ii,:) = traj.a;
    end

    % Initialize the parameters for both inverse and forward dynamics
    params_rne.g = g; % gravity
    params_rne.S = S; % screw axes
    params_rne.M = Mlist; % link frames
    params_rne.G = Glist; % inertial properties
    params_fdyn.g = g; % gravity
    params_fdyn.S = S; % screw axes
    params_fdyn.M = Mlist; % link frames
    params_fdyn.G = Glist; % inertial properties


    % Initialize the (actual) joint variables
    jointPos_actual(:,1) = jointPos_prescribed(:,1);
    jointVel_actual(:,1) = jointVel_actual(:,1);


    for ii = 1 : size(t,2) - 1
        % Calculate the joint torques using the RNE algorithm
        params_rne.jointPos = jointPos_prescribed(:,ii);
        params_rne.jointVel = jointVel_prescribed(:,ii);
        params_rne.jointAcc = jointAcc_prescribed(:,ii);

        

        params_rne.Ftip = zeros(6,1); % end effector wrench for no payload
        %params_rne.Ftip = Ftip; % end effector wrench for payload

        tau_prescribed(:,ii) = rne(params_rne);

        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,ii);
        params_fdyn.jointVel = jointVel_actual(:,ii);
        params_fdyn.tau = tau_prescribed(:,ii);
        params_fdyn.Ftip = zeros(6,1); % end effector wrench for no
        %params_fdyn.Ftip = Ftip; % end effector wrench for payload 

        jointAcc = fdyn(params_fdyn);

        % Integrate the joint accelerations to get velocity and
        % position
        jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
        jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
    end

    tau_prescribed(:,end) = tau_prescribed(:,end-1);
    
    % values needed for plotting:
    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    t_acc = [t_acc t+t(end)*(jj-1)];
    jointVel_acc = [jointVel_acc jointVel_actual];
    jointAcc_acc = [jointAcc_acc jointAcc_prescribed];
    
end

fprintf('\nDone. Simulating the robot...');

%% Animate the robot
title('Inverse Dynamics Control');
robot.plot(jointPos_acc(:,1:100:end)','trail',{'r', 'LineWidth', 2});
fprintf('Done.\n');

%% Plot Joint Positions
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

%% Plot Joint Velocities
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
