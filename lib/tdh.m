function T = tdh(theta, d, a, alpha)
    ct = cos(theta); %calculate the cos of theta
    st = sin(theta); %calculate the sin of theta
    ca = cos(alpha); %calculate the cos of alpha
    sa = sin(alpha); %calculate the sin of alpha

    T = [
         ct   -st*ca   st*sa   a*ct;
         st   ct*ca    -ct*sa  a*st;
         0    sa       ca      d;
         0    0        0       1
         ]; %calculate the homogeneous transformation matrix
end