function [est_target_state, P] = KF_bearing(P, g, state_old, p_c, dt)
% bearing-only
% pseudo linear kalman filter estimator

% state transition matrix
F = [eye(2), dt*eye(2);
    zeros(2), eye(2)];
% pseudo-linear measurement matrix
H=[eye(2)-g*g', zeros(2)];
% pseudolinear measurements
mear_state = (eye(2)-g*g')*p_c;
% noise covariance
Q = 0.000001*diag([0, 0, 1, 1]);
R = 0.0001*diag([1, 1]);
r = norm(p_c - state_old(1:2));
E = eye(2)-g*g';
R = r^2*E*R*E';

% Kalman filter steps
state_old = F * state_old;
P = F*P*F' + Q;
K = P*H'*pinv(H*P*H' + R);
est_target_state = state_old + K*(mear_state - H * state_old);
P = (eye(4)-K*H)*P;

end

