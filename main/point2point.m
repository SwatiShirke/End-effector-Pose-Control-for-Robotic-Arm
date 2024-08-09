function [Tau, Pos, Vel, Acc, Tpass] = point2point(startingPos, targetPos,currentQ, robot,load, Jacobian_method)
g = [0 0 -9.81]'; % Gravity Vector [m/s^2]

% Create a kinematic model of the robot
[S,M] = make_kinematics_model(robot);
n = size(S,2); % read the number of joints
path = [startingPos,targetPos];
nPts = size(path,2);

% Create a dynamical model of the robot
[Mlist,Glist] = make_dynamics_model(robot);

fprintf('Calculating the Inverse Kinematics... ');
%scatter3(startingPos, targetPos, 'filled');
title('Inverse Dynamics Control');

waypoints = zeros(n,2);

ik_method = 'DLS';

for ii = 1:nPts

    targetPose = path(:,ii);
    if strcmp(Jacobian_method,'Jo')
        currentQ = ikinePose(S,M,targetPose,currentQ,ik_method);
    else
        currentQ = ikine(S,M,targetPose,currentQ,ik_method);
    end
    waypoints(:,ii) = currentQ;
end
fprintf('Done.\n');

% Inititalize the variables where we will store the torque profiles, joint
% positions, and time, so that we can display them later
tau_acc = [];
jointPos_acc = [];
jointVel_acc = [];
jointAcc_acc = [];
t_acc = [];

for jj = 1 : nPts - 1

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
    jointAcc_actual = zeros(n,size(t,2)); % Joint Accelerations (Actual)

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
        % end effector wrench
        %params_rne.Ftip = zeros(6,1);
        %params_rne.Ftip = [0 0 -9.81 0 0 0]'; % 1kg payload

        % end effector wrench
        Mload = load;
        T = fkine(S,M,params_rne.jointPos,'space');
        Ftip_inS = [cross(T(1:3,4), Mload * -g); Mload * -g];
        params_rne.Ftip = adjoint(T)' * Ftip_inS;

        tau_prescribed(:,ii) = rne(params_rne);

        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,ii);
        params_fdyn.jointVel = jointVel_actual(:,ii);
        params_fdyn.tau = tau_prescribed(:,ii);
        % end effector wrench
        params_fdyn.Ftip = params_rne.Ftip;
        %params_fdyn.Ftip = [0 0 -9.81 0 0 0]'; % 1kg payload

        jointAcc = fdyn(params_fdyn);

        % Integrate the joint accelerations to get velocity and
        % position
        jointAcc_actual(:,ii+1) = jointAcc;
        jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
        jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
    end

    tau_prescribed(:,end) = tau_prescribed(:,end-1);

    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    jointVel_acc = [jointVel_acc jointVel_actual];
    jointAcc_acc = [jointAcc_acc jointAcc_actual];

    t_acc = [t_acc t+t(end)*(jj-1)];
end

Tau = tau_acc;
Pos = jointPos_acc;
Vel = jointVel_acc;
Acc = jointAcc_acc;
Tpass = t_acc;

end