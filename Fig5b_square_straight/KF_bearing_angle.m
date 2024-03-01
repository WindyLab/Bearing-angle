function est_target_state = KF_bearing_angle(g,theta,state_old, p_c,dt)
%% bearing-angle, Kalman filter estimator

% states transition matrix
F = [eye(2),    dt*eye(2),  zeros(2, 1);
    zeros(2),   eye(2),     zeros(2, 1);
    zeros(1, 2), zeros(1, 2), 1];

% pseudo linear measurement matrix
H=[eye(2)-g*g',     zeros(2),   zeros(2,1);
   theta*eye(2),    zeros(2),   -g];

% pseudo linear measurements
mear_state = [(eye(2)-g*g')*p_c;
              theta*p_c];
          
% noise covariance
Q = 0.000001*diag([0, 0, 1, 1, 0.01]); 
R = 0.0001*diag([1, 1, 1]);
r = norm(p_c - state_old(1:2));
E = [eye(2)-g*g', zeros(2,1);
    theta*eye(2), -g];
R = r^2*E*R*E';


% covariance of estimated states error
persistent P
if isempty(P)
    P =0.1*diag([1, 1, 1, 1, 1]);
end


% Kalman filter steps
state_old = F * state_old;
delta_mear = mear_state - H * state_old;
P = F*P*F' + Q;
K = P*H'*pinv(H*P*H' + R);
est_target_state = state_old + K*delta_mear;
P = (eye(size(K,1))-K*H)*P;

% prevent negtive size
if est_target_state(5) < 0
    est_target_state(5) = 0.1;
end

end

