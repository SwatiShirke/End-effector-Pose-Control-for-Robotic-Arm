function J_b = jacobe(S,M,q)    
    % Number of joints
    n = size(S, 2);

    % Initialize the Jacobian matrix
    J_s = zeros(6, n);

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
        J_s(:, i) = adjoinTwist(Si, T);
    end
    
    T_space = fkine(S,M,q,'space');
    
    R = T_space(1:3, 1:3);
    p = T_space(1:3, 4);
    
    P_skew = [0, -p(3), p(2);
         p(3), 0, -p(1);
        -p(2), p(1), 0];

    Adt = [R' , zeros(3);
          -R' * P_skew, R'];
    
    J_b = Adt*J_s;
end