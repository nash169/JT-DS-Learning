%% Test Robot creation
clear; close all; clc;

% DH parameters - d , theta, aplha, r
% d is the distance along Z_n between the origin of joint n+1 and n [D]
% theta is the angle around Z_n between X_n and X_n+1 (here not defined it is zero the two joints are aligned)
% r is the distance along X_n+1 between the origin of joint n+1 and n (for theta=0 this is the link length) [A]
% alpha id the angle around X_n+1 between Z_n and Z_n+1 [Alpha]
% Q is a vector expressing the joint's limits
% T is vector definig the robot's origin
A = [.4, .4, .2];
D = [0, 0, 0];
Alpha = [0, 0, 0];
Qmin = -pi*[1, 1, -.25];
Qmax = pi*[1, 1, .75];
dimq = size(A,2);

% Define Robot struct. For each arm the first element is the model
% geometry, the second one is the model plant and the third one is the
% joint space trajectory to learn.
robot.arm1 = 'arm1';
robot.arm2 = 'arm2';

% Robot Arm 1 at [-0.4, -0.4, 0]
L = CreateLinks(A,D,Alpha,Qmin,Qmax);
arm1 = SerialLink(L, 'name','arm1', 'base', transl([-0.4, -0.4, 0]));

% Robot Arm 2 at [-0.4, 0.4, 0]
arm2 = SerialLink(arm1, 'name', 'arm2', 'base', transl([-0.4, 0.4, 0]));

% Model plant for the robot's motor controller
robot(1).arm1 = RobotPlant(arm1, 'end_trans');
robot(1).arm2 = RobotPlant(arm2, 'end_trans');

% Plot the robots
robotVisualize = ShowRobot(robot);

%% Trajectories creation & Data Processing

% Get Joint Space target configuration - substitute this function with the
% the inverse kinemetics member method of the robotic toolbox
angles = GetJointAngles([0.4, 0], [-.4,-.4], A, 0);

% Plot obstacles and target
plot([0.3, 0.5], [0.2, 0.2], '-')
plot([0.3, 0.5], [-0.2, -0.2], '-')
plot(0.4,0, 'o')

% Demonstrated trajectories sample rate - 1 second simulation
f = 200;
dt = 1/f;
t = 0 : dt : 1;

% Forward swing trajectory of the footstep in joint space
robot(2).arm1 = [CreateTraj(0, -pi/3, 50, -pi/3, angles(1), 50);
                 CreateTraj(0, pi/2 + .6, 50, pi/2 + .6, angles(2), 50);
                 CreateTraj(0, pi/4, 50, pi/4, angles(3), 50)];
                 
robot(3).arm1 = [CreateTraj(angles(1), -pi/3, 50, -pi/3, 0, 50);
                 CreateTraj(angles(2), pi/2 + .6, 50, pi/2 + .6, 0, 50);
                 CreateTraj(angles(3),pi/4, 50, pi/4, 0, 50)];

robot(2).arm2 = -robot(2).arm1;
robot(3).arm2 = -robot(3).arm1;

% Check generated trajectory
% robotVisualize = PlayTraj(robot, dt, robotVisualize);

tol_cutting = .1;
robot = CreateDataset(robot, 10, dt, tol_cutting);

%% Model Learning

% Options for the JT-DS learner.
options.latent_mapping_type = 'PCA';
options.autoencoder_num_dims = 2;
options.GMM_sigma_type = 'full'; % Can be either 'full' or 'diagonal'
options.explained_variance_threshold = .98; % How much of original data should be explained by latent-space projection
options.GMM_maximize_BIC = false; % If false, always use "options.fixed_num_gaussians" Gaussians in GMM model
options.fixed_num_gaussians = 3;
options.max_gaussians = 8; % Maximum number of Gaussians allowed in learned GMM
options.BIC_regularization = 2.5; % this should probably be between 1 and 3 - the higher it is, the fewer Gaussians will be used
options.verbose = true;
options.learn_with_bounds = true; % If false, does not incorporates joint limits in learning

% Parameters model learning
[Priors, Mu, Sigma, As, latent_mapping] = JTDS_Solver(Data,robotplant,options);

%% Simulate learned trajectory

% Controller creation
motion_generator_learned = MotionGeneratorBounded(robotplant, Mu, Sigma, Priors, As, latent_mapping);

% Controller creation without the learned parameters
[~, identity_mapping] = compute_mapping(eye(dimq), 'None'); % Check this
motion_generator_unlearned = MotionGeneratorBounded(robotplant, zeros(dimq, 1), eye(dimq), 1, eye(dimq), identity_mapping);

% Initial Joint space state
q_initial = [-pi/2-.5; -0.2; pi/2];

% Task positions in target space
x_targets = [.4; -.2; 0];

% How long to interpolate the trajectory - check this (not clear). Reading
% further... it's a time
max_trajectory_duration = 60;

% Maximum distance from the goal
goal_tolerance = 0.05;

% Interpolate the trajectories till the goal's tolerance is reached unless
% the maximum interpolation time exceeds. This is done both for the learned
% and unlearned model.
[Q_traj_learned, T_traj_learned] = computeFullTrajectory(q_initial, x_targets,...
    motion_generator_learned, goal_tolerance, max_trajectory_duration);
[Q_traj_unlearned, T_traj_unlearned] = computeFullTrajectory(q_initial, x_targets,...
    motion_generator_unlearned, goal_tolerance, max_trajectory_duration);

% Compute the Root Mean Square Error - on what? (check this)
rmse_learned = mean(trajectory_error(motion_generator_learned, Data(1:3, :), Data(4:6, :), Data(7:9, :)));
rmse_unlearned = mean(trajectory_error(motion_generator_unlearned, Data(1:3, :), Data(4:6, :), Data(7:9, :)));

%% Visualizing

% Recall the figure object visualizing the robot
if ~ishandle(fig)
    fig = initialize_robot_figure(robot);
else
    figure(fig);
end

% Plot trajectory and robot motion
plot3(x_targets(1,:),x_targets(2,:),x_targets(3,:), 'ro', 'markersize', 20);
PlaybackTrajectory(robotplant, Q_traj_learned, T_traj_learned, fig);

