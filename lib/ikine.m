function outputQ = ikine(S,M,targetPose,currentQ,ik_method)
T = fkine(S,M,currentQ,'space');
currentPose = T(1:3, 4);
iterations = 0;
inputQ = currentQ;
while norm(targetPose - currentPose) > 1e-3
    J_a = jacoba(S,M,currentQ);

    if strcmp(ik_method,'inverse')
        deltaQ = pinv(J_a) * (targetPose - currentPose);

    elseif strcmp (ik_method, 'transpose')
        error_term = targetPose - currentPose;

        alpha = dot(error_term, J_a*J_a'*error_term) / ...
            dot(J_a*J_a'*error_term, J_a*J_a'*error_term);

        deltaQ = alpha * J_a' * (targetPose - currentPose);
    elseif strcmp(ik_method, 'DLS')
        lamda = 0.5;
        deltaQ = J_a' * pinv(J_a*J_a' + lamda^2 *eye(3)) * (targetPose - currentPose);
    else
        error('Wrong IK method')
    end

    currentQ = currentQ + deltaQ';

    T = fkine(S,M,currentQ,'space');
    currentPose = T(1:3,4);

    iterations = iterations + 1;
    if iterations > 500
        message = msgbox("IK Iteration exceeded the maximum threshold, robot will pause","Error","error");
        pause(3);
        delete(message);
        break;
    end
end
if iterations > 500
    outputQ = inputQ;
else
    outputQ = currentQ;
end
end