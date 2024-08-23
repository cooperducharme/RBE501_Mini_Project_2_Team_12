function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the BARRET WAM 7DOF robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

% URDF file: https://git.barrett.com/software/barrett-ros2-pkg/-/blob/devel/src/wam_description/urdf/wrist_link.urdf.xacro
%% Create the manipulator

% Link Lengths in (m)
L1 = 0.550; 
L2 = 0.3;
L3 = 0.06;
W1 = 0.045;

% Link poses when the robot is in the home configuration
    M01 = [1, 0, 0, 0;
           0, 1, 0, 0;
           0, 0, 1, 0;
           0, 0, 0, 1];
    M12 = [1,  0, 0, 0;
           0,  0, 1, 0;
           0, -1, 0, 0;
           0,  0, 0, 1];
    M23 = [1, 0, 0, 0;
           0, 1, 0, 0;
           0, 0, 1, 0;
           0, 0, 0, 1];
    M34 = [1,  0, 0,  W1;
           0,  0, 1,  0;
           0, -1, 0,  L1;
           0,  0, 0,  1];
    M45 = [1,  0, 0,  -W1;
           0,  1, 0,  0;
           0,  0, 1,  L2;
           0,  0, 0,  1];
    M56 = [1,  0, 0,  0;
           0,  0, -1,  0;
           0,  1, 0,  0;
           0,  0, 0,  1];
    M67 = [1,  0, 0,  0;
           0,  0, 1,  0;
           0,  -1, 0,  0;
           0,  0, 0,  1];
    M78 = [1,  0, 0,  0;
           0,  0, 1,  0;
           0,  -1, 0,  L3;
           0,  0, 0,  1];

    Mlist = cat(3,M01, M12, M23, M34, M45, M56, M67, M78);

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
% Define masses of each link (kg)
    m1 = 0.1;
    m2 = 0.1;
    m3 = 0.1;
    m4 = 0.017270635973466;
    m5 = 0.0149005443008753;
    m6 = 0.000506696185792322;
    m7 = 0.0991011928366143;

    % Define inertia matrix of each link
    Ixx1 = 0.13488033;
    Ixy1 = -0.00213041;
    Ixz1 = -0.00012485;
    Iyy1 = 00.11328369;
    Iyz1 = 0.00068555;
    Izz1 = 0.09046330;

    Ixx2 = 0.02140958;
    Ixy2 = 0.00027172;
    Ixz2 = 0.00002461;
    Iyy2 = 0.01377875;
    Iyz2 = -0.00181920;
    Izz2 = 0.01558906;

    Ixx3 = 0.05911077;
    Ixy3 = -0.00249612;
    Ixz3 = 0.00000738;
    Iyy3 = 0.00324550;
    Iyz3 = -0.00001767;
    Izz3 = 0.05927043;

    Ixx4 = 0.0144861530368099;
    Ixy4 = -7.35011796438337E-08;
    Ixz4 = 1.46278041404691E-05;
    Iyy4 = 0.0143994232215456;
    Iyz4 = -0.000349637115920233;
    Izz4 = 0.00465327086275674;

    Ixx5 = 8.68246260680858E-05;
    Ixy5 = 8.32563149485477E-12;
    Ixz5 = 9.56774172507725E-11;
    Iyy5 = 8.70844273808338E-05;
    Iyz5 = 8.15648112659227E-11;
    Izz5 = 6.71124003197135E-05;

    Ixx6 = 0.000369379806854581;
    Ixy6 = 1.50400439239954E-05;
    Ixz6 = -2.41001577391907E-09;
    Iyy6 = 0.000619056939139963;
    Iyz6 = 1.91499423781416E-10;
    Izz6 = 0.000747159541140852;

    Ixx7 = 6.46940974881037E-05;
    Ixy7 = -7.72224623395982E-21;
    Ixz7 = -3.76158192263132E-36;
    Iyy7 = 0.000127725203661566;
    Iyz7 = 2.6027372930156E-21;
    Izz7 = 6.46940974881037E-05;

    I1 = [Ixx1, Ixy1, Ixz1;
          Ixy1, Iyy1, Iyz1;
          Ixz1, Iyz1, Izz1];
    I2 = [Ixx2, Ixy2, Ixz2;
          Ixy2, Iyy2, Iyz2;
          Ixz2, Iyz2, Izz2];
    I3 = [Ixx3, Ixy3, Ixz3;
          Ixy3, Iyy3, Iyz3;
          Ixz3, Iyz3, Izz3];
    I4 = [Ixx4, Ixy4, Ixz4;
          Ixy4, Iyy4, Iyz4;
          Ixz4, Iyz4, Izz4];
    I5 = [Ixx5, Ixy5, Ixz5;
          Ixy5, Iyy5, Iyz5;
          Ixz5, Iyz5, Izz5];
    I6 = [Ixx6, Ixy6, Ixz6;
          Ixy6, Iyy6, Iyz6;
          Ixz6, Iyz6, Izz6];
    I7 = [Ixx7, Ixy7, Ixz7;
          Ixy7, Iyy7, Iyz7;
          Ixz7, Iyz7, Izz7];

    % Define spatial intertia matrices for each link
    G1 = zeros(6,6);
    G1(1:3, 1:3) = I1;
    G1(4:6, 4:6) = m1*eye(3);

    G2 = zeros(6,6);
    G2(1:3, 1:3) = I2;
    G2(4:6, 4:6) = m2*eye(3);

    G3 = zeros(6,6);
    G3(1:3, 1:3) = I3;
    G3(4:6, 4:6) = m3*eye(3);

    G4 = zeros(6,6);
    G4(1:3, 1:3) = I4;
    G4(4:6, 4:6) = m4*eye(3);

    G5 = zeros(6,6);
    G5(1:3, 1:3) = I5;
    G5(4:6, 4:6) = m5*eye(3);

    G6 = zeros(6,6);
    G6(1:3, 1:3) = I6;
    G6(4:6, 4:6) = m6*eye(3);

    G7 = zeros(6,6);
    G7(1:3, 1:3) = I7;
    G7(4:6, 4:6) = m7*eye(3);

    Glist = cat(3,G1, G2, G3, G4, G5, G6, G7);


end