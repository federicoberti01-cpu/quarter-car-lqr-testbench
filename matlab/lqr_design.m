function [K, cl_ss] = lqr_design(sys,q1,q2,q3,r)
    % Performs the design on an LQR controller for the plant. 
    % Accepts state weights q1,q2,q3 and input weight r (q4 is already encoded in r). 
    % Returns the state feedback matrix K and the closed loop system.
    % The function checks for controllability before defining a diagonal Q
    % and R costs based on the value of weights q1,...,q3(for states 1-3 +
    % input).
    % It neglects disturbance column.

    A = sys.A;
    Bu = sys.B(:, 1);
    D = sys.D;
    C = sys.C;              
    Du = D(:,1);                    % Nonzero entries in D
    
    if rank(ctrb(A, Bu)) < size(A, 1)
        error('The plant is not controllable.');
    end
    
    % Cost Matrices
    W = diag([q1,q2,q3,0]);         % Cost diagonal matrix
    Q = C'*W*C;
    R = Du'*W*Du + r;
    N = C'*W*Du;
    K = lqr(A, Bu, Q, R, N);

    % Form the closed-loop state-space system using state feedback.
    A_cl = A-Bu*K;
    B_cl = sys.B(:,2);  % After closing the loop, the remaining input is only the disturbance 
    C_cl = C-Du*K;
    D_cl = D(:,2);  % Remaining column of D representing road input

    cl_ss = ss(A_cl,B_cl,C_cl,D_cl);
end
