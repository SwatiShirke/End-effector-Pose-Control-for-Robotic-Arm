function R = axisangle2rot(omega,theta)
    % Extract components of omega
    w1 = omega(1);
    w2 = omega(2);
    w3 = omega(3);
    
    % Compute skew-symmetric matrix Omega
    Omega = [0, -w3, w2;
             w3, 0, -w1;
             -w2, w1, 0];
    
    % Compute rotation matrix using Rodrigues' formula
    R = eye(3) + sin(theta) * Omega + (1 - cos(theta)) * Omega^2;
end