function robot = make_robot()
%   Inputs: none
%
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox
%
%   Author: Ao Jiang
%   Last modified: 4/29/2024

%% Create the manipulator
d1 = 330e-3;
d4 = -320e-3;
d6 = -80e-3;

a1 = 75e-3;
a2 = 300e-3;
a3 = -75e-3;


robot = SerialLink([Revolute('d', d1, 'a', a1, 'alpha', -pi/2), ...
                    Revolute('d', 0,  'a', a2, 'alpha', pi, 'offset', -pi/2), ...
                    Revolute('d', 0,  'a', a3, 'alpha', pi/2, 'offset', pi), ...
                    Revolute('d', d4, 'a', 0,  'alpha', -pi/2), ...
                    Revolute('d', 0,  'a', 0, 'alpha', pi/2), ...
                    Revolute('d', d6,  'a', 0, 'alpha', pi, 'offset',pi)], ...
                    'name', 'Fanuc LR Mate 200iC'); 


end

