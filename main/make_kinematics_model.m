function [S,M] = make_kinematics_model(robot)
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% the Fanuc LR Mate 200iC robot.
%
% Inputs: robot - the robot object created by the robotics toolbox
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

d1 = 330e-3;
d4 = 320e-3;

a1 = 75e-3;
a2 = 300e-3;
a3 = 75e-3;

% Screw Axes
S = [0 0 1 0 0 0;
      0  1  0 -cross([ 0  1  0], [a1 0 d1]);
      0 -1  0 -cross([ 0 -1  0], [a1 0 d1+a2]);
     -1  0  0 -cross([-1  0  0], [0 0 d1+a2+a3]);
      0 -1  0 -cross([ 0 -1  0], [a1+d4 0 d1+a2+a3]);
     -1  0  0 -cross([-1  0  0], [0 0 d1+a2+a3])]';

% Home configuration
M = double(robot.fkine(zeros(1,6)));
R = M(1:3,1:3);
%eulXYZ = rotm2eul(R,'XYZ')

end