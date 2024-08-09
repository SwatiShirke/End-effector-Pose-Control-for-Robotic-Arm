function twist_inB = adjoinTwist(twist_inA,T_AB)
    R = T_AB(1:3, 1:3);
    p = T_AB(1:3, 4);
    
    P = [0, -p(3), p(2);
         p(3), 0, -p(1);
        -p(2), p(1), 0];

    Adt = [R  zeros(3,3);
           P*R  R];
    
    twist_inB = Adt*twist_inA;
end
