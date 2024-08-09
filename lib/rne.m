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
% Extract parameters
g = params.g;
S = params.S;
M = params.M;
G = params.G;
q = params.jointPos;
qd = params.jointVel;
qdd = params.jointAcc;

% Number of joints
n = size(S, 2);

% Initialize output variables
tau = zeros(n, 1);
V = zeros(6, n+1);
Vdot = zeros(6, n+1);

% Calculate the tranformation matrix between joints and base frame
MT = zeros(4, 4, n+1);
MT(:,:,1) = M(:, :, 1);
for i = 1:n
    MT(:,:,i+1) = MT(:,:,i)*M(:, :, i+1);
end

% Calculate A axis
A = zeros(6,n);
for i = 1:n
    A(:,i) = adjoint(inv(MT(:,:,i)))*S(:,i);
end

Vdot(:,1) = [0 0 0 -g']';
% Forward iterations
for i = 1:n
    % Calculate the transformation
    T = M(:, :, i)*twist2ht(A(:,i),q(i));

    % Update the twist of the current link
    V(:,i+1) = A(:,i) * qd(i) + adjoint(inv(T))*V(:,i);

    % Update the acceleration of the current link
    Vdot(:,i+1) = A(:,i) * qdd(i) + adjoint(inv(T))*Vdot(:,i) + ad(V(:,i+1))*A(:,i)*qd(i);
end

% Backward iterations
W = zeros(6,n+1);
W(:,n+1) = params.Ftip;

for i = n:-1:1

    % No joint value for end-effector
    if i == n
        T = M(:, :, i+1);
    else
        T = M(:, :, i+1)*twist2ht(A(:,i+1),q(i+1));
    end

    % Update W
    W(:,i) = G(:, :, i)*Vdot(:, i+1) - ad(V(:,i+1))'*G(:, :, i)*V(:,i+1) + adjoint(inv(T))'*W(:,i+1);

    % Calculate Tau
    tau(i) = W(:,i)' * A(:, i);
end
end