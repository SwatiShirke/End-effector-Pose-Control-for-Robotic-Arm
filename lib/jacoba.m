function J_a = jacoba(S,M,q)    
    J = jacob0(S,q);
    Jw = J(1:3,:);
    Jv = J(4:6,:);
    
    T = fkine(S,M,q,'space');
    
    Pos = T(1:3,4);
    
    J_a = Jv - skew(Pos')*Jw;
end