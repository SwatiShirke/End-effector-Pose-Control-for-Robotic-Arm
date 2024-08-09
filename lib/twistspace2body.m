function V_b = twistspace2body(V_s,T)
    R = T(1:3, 1:3); % rotational part
    p = T(1:3, 4);   % translational part

    P_skew = [0, -p(3), p(2);
         p(3), 0, -p(1);
        -p(2), p(1), 0];
    
    Adt = [R' , zeros(3);
          -R' * P_skew, R'];
    
    V_b = Adt * V_s;
end
