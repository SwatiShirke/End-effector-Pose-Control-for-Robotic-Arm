function T = twist2ht(S,theta)
    omega = S(1:3);  % Angular velocity
    v = S(4:6);      % Linear velocity
    
    % Calculate rotation matrix
    R = axisangle2rot(omega, theta);
    
    % Compute skew-symmetric matrix Omega
    Omega = [0, -omega(3), omega(2);
             omega(3), 0, -omega(1);
             -omega(2), omega(1), 0];
    
    % Construct homogeneous transformation matrix
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = (eye(3)*theta + (1-cos(theta))*Omega + (theta - sin(theta))*Omega^2) * v;
end