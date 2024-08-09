function [M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot)
  Mj1 = tdh(robot.offset(1), robot.d(1), robot.a(1), robot.alpha(1));
  Mj2 = Mj1 * tdh(robot.offset(2), robot.d(2), robot.a(2), robot.alpha(2));
  Mj3 = Mj2 * tdh(robot.offset(3), robot.d(3), robot.a(3), robot.alpha(3));
  Mj4 = Mj3 * tdh(robot.offset(4), robot.d(4), robot.a(4), robot.alpha(4));
  Mj5 = Mj4 * tdh(robot.offset(5), robot.d(5), robot.a(5), robot.alpha(5));
  Mj6 = Mj5 * tdh(robot.offset(6), robot.d(6), robot.a(6), robot.alpha(6));
  
  M1 = Mj1 * [eye(3) [0.105 0.09 0.08042]'; 0 0 0 1];
  M2 = Mj2 * [eye(3) [0.0 0.0 0.081]'; 0 0 0 1];
  M3 = Mj3 * [eye(3) [0 -0.00155 0.17556]'; 0 0 0 1];
  M4 = Mj4 * [eye(3) [0.008 0.00068 0.0177]'; 0 0 0 1];
  M5 = Mj5 * [eye(3) [0.24013 0 0]'; 0 0 0 1];
  M6 = Mj6 * [eye(3) [0.01573 0.003 -0.00006]'; 0 0 0 1];
  
  M01 = M1;
  M12 = pinv(pinv(M2)*M1);
  M23 = pinv(pinv(M3)*M2);
  M34 = pinv(pinv(M4)*M3);
  M45 = pinv(pinv(M5)*M4);
  M56 = pinv(pinv(M6)*M5);
  M67 = eye(4);
end

