function x = inverse_state_transform(zeta, L)
    % INVERSE_STATE_TRANSFORM Transforms feedback linearized state into original state.
    %
    %   X = INVERSE_STATE_TRANSFORM(ZETA, L) Transforms feedback linearized
    % state ZETA into original state of the robot X. L is the length of
    % the robots' "hand". L can be a vector with the same number of 
    % elements as robots, or a scalar.
    %
    % See also: STATE_TRANSFORM
    
    N = int32(numel(zeta)/5);
    if isscalar(L) && N>1
        L = L * ones(N,1);
    end
    
    x = zeros(5*N, 1);
    
    for k = 0:(N-1)
        theta = zeta(5*k+5);

        c = cos(theta);
        s = sin(theta);

        x(5*k+1) = zeta(5*k+1) - L(k+1)*c;
        x(5*k+2) = zeta(5*k+2) - L(k+1)*s;
        x(5*k+3) = theta;
        x(5*k+4) = (zeta(5*k+3)*c + zeta(5*k+4)*s)/2;
        x(5*k+5) = (-zeta(5*k+3)*s + zeta(5*k+4)*c)/(2*L(k+1));
    end
end
