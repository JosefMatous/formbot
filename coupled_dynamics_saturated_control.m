function v = coupled_dynamics_saturated_control(zeta, h_ref, k, kg, d, kf)
    % COUPLED_DYNAMICS_SATURATED_CONTROL Implements saturated control law
    %
    %   V = COUPLED_DYNAMICS_SATURATED_CONTROL(ZETA, H_REF, K, KG, D, KF)
    % Returns a vector of control inputs V using formula (16). ZETA are the
    % transformed states, H_REF is the robots' "hand" position reference,
    % K, KG, and D are positive scalars, and KF is a non-negative scalar.
    %
    % See also: COUPLED_DYNAMICS_CONTROL
    
    N = numel(zeta) / 5;
    
    position_indices = kron(0:(N-1), [5 5]) + repmat([1,2],1,N);
    velocity_indices = kron(0:(N-1), [5 5]) + repmat([3,4],1,N);
    
    H = zeta(position_indices) - h_ref;
    Hdot = zeta(velocity_indices);
    
    if N > 1
        tanh_prev = kf*tanh(k*(H - [H(end-1:end); H(1:end-2)]));
        tanh_next = kf*tanh(k*(H - [H(3:end); H(1:2)]));
    else
        tanh_prev = zeros(2,1);
        tanh_next = zeros(2,1);
    end
    
    v = - kg*tanh(k*H) - d*tanh(k*Hdot) - tanh_prev - tanh_next;
end
