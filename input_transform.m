function u = input_transform(v, x, m, J, L)
    % INPUT_TRANSFORM Transforms the linearized input into original input.
    %
    %   U = INPUT_TRANSFORM(V, X, M, J, L) Transforms the linearized input V 
    % into input U of the original system. X is the state of the system, m 
    % is the mass of the robot, J is the moment of inertia, L is the length
    % of the robots' "hand". M, J and L can be either vectors with the same
    % number of elements as robots, or scalars.
    %
    % See also: STATE_TRANSFORM
    
    N = int32(numel(v)/2);
    if N>1
        if isscalar(m)
            m = m * ones(N, 1);
        end
        if isscalar(J)
            J = J * ones(N, 1);
        end
        if isscalar(L)
            L = L * ones(N,1);
        end
    end
    
    u = zeros(2*N, 1);
    for k = 0:(N-1)
        theta = x(5*k+3);
        V = x(5*k+4);
        omega = x(5*k+5);
        c = cos(theta);
        s = sin(theta);

        A = [c/m(k+1), -L(k+1)*s/J(k+1); s/m(k+1), L(k+1)*c/J(k+1)];

        u(2*k+1:2*k+2) = A \ (v(2*k+1:2*k+2) - [-V*omega*s - L(k+1)*omega^2*c; V*omega*c - L(k+1)*omega^2*s]);
    end
end
