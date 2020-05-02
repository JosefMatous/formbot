function v = coupled_dynamics_control(zeta, h_ref, Kg, Dg, Kf, Df)
    % COUPLED_DYNAMICS_CONTROL Implements the "Coupled dynamics" control law
    %
    %   V = COUPLED_DYNAMICS_CONTROL(ZETA, H_REF, KG, DG, KF, DF) Returns a
    % vector of control inputs V using formula (8). ZETA are the
    % transformed states, H_REF is the robots' "hand" position reference,
    % KG and DG are 2-by-2 positive definite matrices, and KF and DF are
    % 2-by-2 positive semidefinite matrices.
    %
    % See also: COUPLED_DYNAMICS_SATURATED_CONTROL
    
    N = numel(zeta) / 5;
    
    position_indices = kron(0:(N-1), [5 5]) + repmat([1,2],1,N);
    velocity_indices = kron(0:(N-1), [5 5]) + repmat([3,4],1,N);
    
    H = zeta(position_indices) - h_ref;
    Hdot = zeta(velocity_indices);
    
    C_row = [2, -1, zeros(1,max(N-3,0)), -1];
    if numel(C_row) > N
        C_row = C_row(1:N);
    end
    C = toeplitz(C_row);
    
    v = - (kron(eye(N),Kg) + kron(C,Kf))*H - (kron(eye(N),Dg) + kron(C,Df))*Hdot;
end
