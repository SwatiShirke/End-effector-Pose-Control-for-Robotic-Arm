function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the Fanuc LR Mate 200iC robot
% 
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link
%
% URDF: https://github.com/sezan92/Fanuc/blob/master/urdf/Fanuc.URDF

%% Link poses when the robot is in the home configuration
[M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
G1 = zeros(6,6);
G2 = zeros(6,6);
G3 = zeros(6,6);
G4 = zeros(6,6);
G5 = zeros(6,6);
G6 = zeros(6,6);

m1 = 4.85331;
m2 = 4.85331;
m3 = 7.96646;
m4 = 2.401;
m5 = 2.89084;
m6 = 0.043284;

G1(4:6, 4:6) = m1*eye(3);
G2(4:6, 4:6) = m2*eye(3);
G3(4:6, 4:6) = m3*eye(3);
G4(4:6, 4:6) = m4*eye(3);
G5(4:6, 4:6) = m5*eye(3);
G6(4:6, 4:6) = m6*eye(3);

G1(1:3,1:3) = [0.000533781           0.00012915752       -0.002807595;
               0                     0.055445798         -0.000094967;
               0                     0                    0.02302818052];

G2(1:3,1:3) = [0.000533781           0.00012915752       -0.002807595;
               0                     0.055445798         -0.000094967;
               0                     0                    0.02302818052];

G3(1:3,1:3) = [0.44125720672         0.000023323         -0.01624821395;
               0                     0.42103008445       -0.00127983617;
               0                     0                    0.03658588308];

G4(1:3,1:3) = [0.00709493364         0.000062924          0.000131383;
               0                     0.08303386           0.000095227;
               0                     0                    0.007482747];

G5(1:3,1:3) = [0.004863267           0.00303203667        0.000018846079;
               0                     0.19336297539       -0.00000096841;
               0                     0                    0.19385148073];

G6(1:3,1:3) = [0.00029985018        -0.00000439197       -0.000000345;
               0                     0.000661879         -0.00000008651;
               0                     0                    0.00064231094];


Glist = cat(3, G1, G2, G3, G4, G5, G6);

end