function K = lqr_design(sys,q1,q2,q3,r)
    % Performs the design on an LQR controller for the plant. 
    % Accepts a state cost matrix Q and an actuator cost matrix R. 
    % Returns the state feedback matrix K.
    % The function checks for controllability before defining a diagonal Q
    % and R costs based on the value of weights q1,...,q3(for states 1-3 +
    % input).
    % It neglects disturbance column.

    A = sys.A;
    B = sys.B(:, 1);
    Cp = sys.C(1:3,:);              % Nonzero entries in C
    Dp = D(:,1);                    % Nonzero entries in D
    
    if rank(ctrb(A, B)) < size(A, 1)
        error('The plant is not controllable.');
    end
    
    % Cost Matrices
    W = diag(q1,q2,q3);         % Cost diagonal matrix
    Q = Cp'*W*Cp;
    R = Dp'*W*Dp + r;
    N = Cp'*W*Dp;
    K = lqr(A, B, Q, R, N);
end
