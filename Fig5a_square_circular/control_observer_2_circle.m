function sc_new = control_observer_2_circle(v, r, state_t,state_c,dt)
% observer, circular motion control
delta_p = state_c(1:2, 1) - state_t(1:2, 1);
if norm(delta_p) > r*1.02
    v_new = -delta_p * v / norm(delta_p);
    p_new = state_c(1:2, 1) + v_new*dt;
else
    theta = -v*dt /r;
    M = [cos(theta), -sin(theta);
        sin(theta), cos(theta)];
    delta_p_new = M*delta_p;
    delta_p_new = delta_p_new * r/norm(delta_p_new);
    p_new = delta_p_new + state_t(1:2, 1);
    v_new = (p_new - state_c(1:2, 1))/dt;
    
end
sc_new = [p_new; v_new];
end

