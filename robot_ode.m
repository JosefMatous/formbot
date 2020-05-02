function dx = robot_ode(x, u, m, J)
    % ROBOT_ODE Ordinary differential equations of robot dynamics
    %
    %   DX = ROBOT_ODE(X, U, M, J) Returns a 5*N-by-1 vector of time
    % derivatives where N is the number of robots. U is a 
    % 2*N-by-1 vector of inputs (Ui = [Fi; Ti]), M are the masses of
    % robots, and J are their moments of inertia. M and J can be vectors
    % with different parameters for each robot, or scalars.
    
    N = int32(numel(x)/5);
    if N>1
        if isscalar(m)
            m = m * ones(N, 1);
        end
        if isscalar(J)
            J = J * ones(N, 1);
        end
    end
    
    dx = zeros(5*N, 1);
    for k = 0:(N-1)
        theta = x(5*k+3);
        v = x(5*k+4);
        omega = x(5*k+5);

        dx(5*k+1) = v*cos(theta);
        dx(5*k+2) = v*sin(theta);
        dx(5*k+3) = omega;
        dx(5*k+4) = u(2*k+1)/m(k+1);
        dx(5*k+5) = u(2*k+2)/J(k+1);
    end
end
