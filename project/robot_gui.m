function varargout = robot_gui(varargin)
% ROBOT_GUI MATLAB code for robot_gui.fig
%      ROBOT_GUI, by itself, creates a new ROBOT_GUI or raises the existing
%      singleton*.
%
%      H = ROBOT_GUI returns the handle to a new ROBOT_GUI or the handle to
%      the existing singleton*.
%
%      ROBOT_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOT_GUI.M with the given input arguments.
%
%      ROBOT_GUI('Property','Value',...) creates a new ROBOT_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before robot_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to robot_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help robot_gui

% Last Modified by GUIDE v2.5 29-Apr-2024 08:41:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @robot_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @robot_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before robot_gui is made visible.
function robot_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to robot_gui (see VARARGIN)

% Choose default command line output for robot_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes robot_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = robot_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

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
% q = zeros(1,7);  % Home position
%robot.plot(q);

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

dt = 1e-2;        % simulation time step [s]
t = 0 : dt : 0.5; % total simulation time [s]

qt = zeros(n,size(t,2));  qt(:,1) = params.jointPos;
qdt = zeros(n,size(t,2)); qdt(:,1) = params.jointVel;

% Display the robot
%robot.plot(params.jointPos');

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
axes(handles.robot_plot)
title('Gravity Compensation');
robot.plot(qt(:,1:100:end)');
fprintf('Done.\n');

%% Plot Joint Positions
axes(handles.pos_plot)
cla
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

%% Display the Joint Velocities
axes(handles.vel_plot)
cla
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

%% Display the Joint Accelerations
axes(handles.acc_plot)
cla
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


%% Display the Joint Torques
axes(handles.torq_plot)
cla
hold on, grid on
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

fprintf('Done.\n');




function pc_x_Callback(hObject, eventdata, handles)
% hObject    handle to pc_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pc_x as text
%        str2double(get(hObject,'String')) returns contents of pc_x as a double


% --- Executes during object creation, after setting all properties.
function pc_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pc_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pc_y_Callback(hObject, eventdata, handles)
% hObject    handle to pc_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pc_y as text
%        str2double(get(hObject,'String')) returns contents of pc_y as a double


% --- Executes during object creation, after setting all properties.
function pc_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pc_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pc_z_Callback(hObject, eventdata, handles)
% hObject    handle to pc_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pc_z as text
%        str2double(get(hObject,'String')) returns contents of pc_z as a double


% --- Executes during object creation, after setting all properties.
function pc_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pc_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pc_roll_Callback(hObject, eventdata, handles)
% hObject    handle to pc_roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pc_roll as text
%        str2double(get(hObject,'String')) returns contents of pc_roll as a double


% --- Executes during object creation, after setting all properties.
function pc_roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pc_roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pc_pitch_Callback(hObject, eventdata, handles)
% hObject    handle to pc_pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pc_pitch as text
%        str2double(get(hObject,'String')) returns contents of pc_pitch as a double


% --- Executes during object creation, after setting all properties.
function pc_pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pc_pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pc_yaw_Callback(hObject, eventdata, handles)
% hObject    handle to pc_yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pc_yaw as text
%        str2double(get(hObject,'String')) returns contents of pc_yaw as a double


% --- Executes during object creation, after setting all properties.
function pc_yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pc_yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function external_force_Callback(hObject, eventdata, handles)
% hObject    handle to external_force (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of external_force as text
%        str2double(get(hObject,'String')) returns contents of external_force as a double


% --- Executes during object creation, after setting all properties.
function external_force_CreateFcn(hObject, eventdata, handles)
% hObject    handle to external_force (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
addpath('../lib');
addpath('../mr');

%Position control

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
%q = zeros(1,7);  % Home position
%robot.plot(q);

%% Kinematics and Dynamics Inputs

% Create a kinematic model of the robot
[S,M] = make_kinematics_model();
n = size(S,2); % read the number of joints

% Create a dynamical model of the robot %Mlist includes end-eff frame
[Mlist,Glist] = make_dynamics_model();

% Bringing in path/point values from gui
my_x = str2double(get(handles.pc_x,'string'));
my_y = str2double(get(handles.pc_y,'string'));
my_z = str2double(get(handles.pc_z,'string'));
my_roll = str2double(get(handles.pc_roll,'string'));
my_pitch = str2double(get(handles.pc_pitch,'string'));
my_yaw = str2double(get(handles.pc_yaw,'string'));
Ftip = str2num(get(handles.external_force,'string'))';


%% Control the motion of the robot between 2 set points
fprintf('----------------------Dynamic Control of a 3-DoF Arm--------------------\n');

% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

path = [0 0 0.91; my_x my_y my_z]'; %where [0 0 0.91] is the home config

rpy = [my_roll my_pitch my_yaw];

nPts = size(path,2);

fprintf('Calculating the Inverse Kinematics... ');
% robot.plot(zeros(1,n)); hold on;
% scatter3(path(1,:), path(2,:), path(3,:), 'filled');
% title('Inverse Dynamics Control');

% Calculate the inverse kinematics
waypoints = zeros(n,nPts);

for ii = 2:nPts %change 1 to 2 when starting from the home configuration

    
    R = eul2rotm(rpy);
    p = path(:,ii);

    T = [R p; 0 0 0 1]; %creating T with target xyz and rpy
    targetPose = MatrixLog6(T);
    targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';
    q0 = zeros(1,n);    
    %waypoints(:,ii) = ikin(S,M,q0,targetPose);
    waypoints(:,ii) = robot.ikine(T);
end


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
        
        %params_rne.Ftip = zeros(6,1); % end effector wrench for no payload
        params_rne.Ftip = Ftip; % end effector wrench for payload
        tau_prescribed(:,ii) = rne(params_rne);

        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,ii);
        params_fdyn.jointVel = jointVel_actual(:,ii);
        params_fdyn.tau = tau_prescribed(:,ii);
        %params_fdyn.Ftip = zeros(6,1); % end effector wrench for no
        params_fdyn.Ftip = Ftip; % end effector wrench for payload 

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
axes(handles.robot_plot)
title('Inverse Dynamics Control');
robot.plot(jointPos_acc(:,1:100:end)','trail',{'r', 'LineWidth', 2});
fprintf('Done.\n');


%% Plot Joint Positions
axes(handles.pos_plot)
cla
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

%% Display the Joint Velocities
axes(handles.vel_plot)
cla
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

%% Display the Joint Accelerations
axes(handles.acc_plot)
cla
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


%% Display the Joint Torques
axes(handles.torq_plot)
cla
hold on, grid on
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

fprintf('Program completed successfully.\n');
