function J = jacob0(S,q) 
    % Number of joints
    n = size(S, 2);

    % Initialize the Jacobian matrix
    J = zeros(6, n);

    % Initialize the transformation matrix to the identity
    T = eye(4);

    % Loop through each joint
    for i = 1:n
        % Extract the screw axis for the current joint
        Si = S(:, i);

        % Calculate the homogeneous transformation matrix for the current joint
        Ti = twist2ht(Si, q(i));

        % Update the overall transformation matrix
        T = T * Ti;

        % Calculate the columns of the Jacobian matrix
        J(:, i) = adjoinTwist(Si, T);
    end
end