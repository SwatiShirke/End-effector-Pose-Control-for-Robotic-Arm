function T = fkine(S,M,q,frame)

% Initialize transformation matrix
T = eye(4);

% Iterate over each joint
for i = 1:length(q)
    Ti = twist2ht(S(:,i),q(i));
    T = T*Ti;
end

if strcmp(frame, 'space')
    T = T*M;
elseif strcmp(frame, 'body')
        T = M*T;
else
    disp("invalid input");
end

end
