function outputQ = ikinePose(S,M,targetPose,currentQ,ik_method)
T = fkine(S,M,currentQ,'space');
currentPose = MatrixLog6(T);
currentPose = [currentPose(3,2) ...
    currentPose(1,3) ...
    currentPose(2,1) ...
    currentPose(1:3,4)']';

iterations = 0;
inputQ = currentQ;
while norm(targetPose - currentPose) > 1e-3
    J = jacob0(S,currentQ);

    if strcmp(ik_method,'inverse')
        deltaQ = pinv(J) * (targetPose - currentPose);

    elseif strcmp (ik_method, 'transpose')
        error_term = targetPose - currentPose;

        alpha = dot(error_term, J*J'*error_term) / ...
            dot(J*J'*error_term, J*J'*error_term);

        deltaQ = alpha * J' * (targetPose - currentPose);
    elseif strcmp(ik_method, 'DLS')
        lamda = 0.5;
        deltaQ = J' * pinv(J*J' + lamda^2 *eye(6)) * (targetPose - currentPose);
    else
        error('Wrong IK method')
    end

    currentQ = currentQ + deltaQ';

    T = fkine(S,M,currentQ,'space');
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
        currentPose(1,3) ...
        currentPose(2,1) ...
        currentPose(1:3,4)']';

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