function AdT = adjoint(T)
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    P = [0, -p(3), p(2);
         p(3), 0, -p(1);
        -p(2), p(1), 0];
    
    AdT = [R  zeros(3,3);
           P*R  R];
end