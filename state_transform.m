function zeta = state_transform(x, L)
    % STATE_TRANSFORM Transforms original robot state into feedback linearized state.
    %
    %   ZETA = STATE_TRANSFORM(X, L) Transforms original state X into a new
    % state ZETA. L is the length of the robots' "hand". L can be a vector
    % with the same number of elements as robots, or a scalar.
    %
    % See also: INPUT_TRANSFORM
    
    N = int32(numel(x)/5);
    if isscalar(L) && N>1
        L = L * ones(N,1);
    end
    
    zeta = zeros(5*N, 1);
    
    for k = 0:(N-1)
        rx = x(5*k+1);
        ry = x(5*k+2);
        theta = x(5*k+3);
        v = x(5*k+4);
        omega = x(5*k+5);

        c = cos(theta);
        s = sin(theta);

        zeta(5*k+1) = rx + L(k+1)*c; 
        zeta(5*k+2) = ry + L(k+1)*s; 
        zeta(5*k+3) = v*c - L(k+1)*omega*s; 
        zeta(5*k+4) = v*s + L(k+1)*omega*c;
        zeta(5*k+5) = theta;
    end
end
