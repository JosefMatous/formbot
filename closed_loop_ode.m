function dx = closed_loop_ode(t, x, h_ref, param)
    % CLOSED_LOOP_ODE Implements ODEs of the closed-loop system
    %
    %   DX = CLOSED_LOOP_ODE(T, X, H_REF, PARAM) Returns a vector of time
    % derivatives of state X. T is simulation time, H_REF is the matrix of
    % waypoints. PARAM is a structure with parameters. PARAM must always
    % have a field named "mode", which should be a member of the
    % <a href="matlab: doc ControlMode">ControlMode</a> enumeration, a field "m", which is the robots' mass, a field "J", 
    % which is the robots' moment of inertia, a field "L", which is the
    % length of the robots' hand, and a field "h_tol" which specifies the
    % goal tolerance (if |H-H_REF|<h_tol, the robots move to next waypoint. 
    % Other fields of the structure depend on the selected mode:
    %  * Default (Coupled dynamics formation control) requires
    %    - Kg, Dg : 2-by-2 symmetric positive definite matrices
    %    - Kf, Df : 2-by-2 symmetric positive semidefinite matrices
    %  * Damping (Coupled dynamics control with passivity-based damping)
    %    - Kg, D : 2-by-2 symmetric positive definite matrices
    %    - Kf : 2-by-2 positive semidefinite matrix
    %    - A : 2-by-2 Hurwitz matrix
    %    - P : positive definite solution to A^T.P + P^T.A + Q = 0
    %  * Saturation (Saturated control)
    %    - kf : scalar >=0
    %    - kg, k, d : scalars >0
    
    persistent wp % waypoint index
    if isempty(wp) || t == 0
        wp = 1;
    end
    
    switch param.mode
        case ControlMode.Default
            zeta = state_transform(x, param.L);
            v = coupled_dynamics_control(zeta, h_ref(:,wp), param.Kg, param.Dg, param.Kf, param.Df);
            u = input_transform(v, x, param.m, param.J, param.L);
            dx = robot_ode(x, u, param.m, param.J);
            
            N = numel(x)/5;
            position_indices = kron(0:(N-1), [5 5]) + repmat([1,2],1,N);
            H = zeta(position_indices) - h_ref(:,wp);
        case ControlMode.Damping
            N = numel(x)/7;
            xx = x(1:5*N);
            x_hat = x(5*N+1:end);
            zeta = state_transform(xx, param.L);
            v = coupled_dynamics_control(zeta, h_ref(:,wp), param.Kg + param.P, ...
                param.D, param.Kf, zeros(2)) - kron(eye(N), param.P*param.A)*x_hat;
            u = input_transform(v, xx, param.m, param.J, param.L);
            
            position_indices = kron(0:(N-1), [5 5]) + repmat([1,2],1,N);
            H = zeta(position_indices) - h_ref(:,wp);
            dx = [robot_ode(xx, u, param.m, param.J); 
                  kron(eye(N), param.A) * x_hat + H];
        case ControlMode.Saturation
            zeta = state_transform(x, param.L);
            v = coupled_dynamics_saturated_control(zeta, h_ref(:,wp), param.k, param.kg, param.d, param.kf);
            u = input_transform(v, x, param.m, param.J, param.L);
            dx = robot_ode(x, u, param.m, param.J);
            
            N = numel(x)/5;
            position_indices = kron(0:(N-1), [5 5]) + repmat([1,2],1,N);
            H = zeta(position_indices) - h_ref(:,wp);
    end
    
    if norm(H) < param.h_tol && wp < size(h_ref, 2)
        wp = wp + 1;
    end
end
