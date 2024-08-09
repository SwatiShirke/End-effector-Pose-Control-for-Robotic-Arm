function traj = make_trajectory(type, params)
% Extracting parameters
t0 = params.t(1);
tf = params.t(2);
dt = params.dt;
q0 = params.q(1);
qf = params.q(2);
v0 = params.v(1);
vf = params.v(2);

% Calculating time vector
t = t0:dt:tf;

% Generating trajectory based on type
switch type
    case 'cubic'
        B = [q0; v0; qf; vf];

        M = [1  t0  t0^2  t0^3;
            0  1   2*t0  3*t0^2;
            1  tf  tf^2  tf^3;
            0  1   2*tf  3*tf^2
            ];

        A = pinv(M)*B;

        q = A(1) + A(2) * (t - t0) + A(3) * (t - t0).^2 + A(4) * (t - t0).^3;
        v = A(2) + 2 * A(3) * (t - t0) + 3 * A(4) * (t - t0).^2;
        a = 2 * A(3) + 6 * A(4) * (t - t0);

    case 'quintic'
        % Additional parameters for quintic polynomial trajectory
        a0 = params.a(1);
        af = params.a(2);

        B = [q0; v0; a0; qf; vf; af];

        M = [ 1 t0 t0^2 t0^3 t0^4 t0^5;
            0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
            0 0 2 6*t0 12*t0^2 20*t0^3;
            1 tf tf^2 tf^3 tf^4 tf^5;
            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
            0 0 2 6*tf 12*tf^2 20*tf^3];

        A = pinv(M) * B;

        q = A(1) + A(2) * (t - t0) + A(3) * (t - t0).^2 + A(4) * (t - t0).^3 + A(5) * (t - t0).^4 + A(6) * (t - t0).^5;
        v = A(2) + 2 * A(3) * (t - t0) + 3 * A(4) * (t - t0).^2 + 4 * A(5) * (t - t0).^3 + 5 * A(6) * (t - t0).^4;
        a = 2 * A(3) + 6 * A(4) * (t - t0) + 12 * A(5) * (t - t0).^2 + 20 * A(6) * (t - t0).^3;


    otherwise
        error('Invalid trajectory type.');
end

traj.t = t;
traj.q = q;
traj.v = v;
traj.a = a;
end