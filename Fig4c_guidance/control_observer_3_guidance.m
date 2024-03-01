function sc_new = control_observer_3_guidance(v, state_t,state_c,dt)
% observer, guidance control
k = 1;
g = state_t(1:2, 1) - state_c(1:2, 1);
angle_g_now = atan2(g(2), g(1));
persistent angle_g_old angle_v_old
if isempty(angle_g_old)
    angle_g_old = angle_g_now;
    angle_v_old = atan2(g(2), g(1));
    sc_new(3:4, 1) = v*[cos(angle_v_old); sin(angle_v_old)];
    sc_new(1:2, 1) = state_c(1:2, 1) + sc_new(3:4, 1)*dt;
    return
end
r = norm(g);
if r < 0.1  
    v = 0;
end

rate_g = (angle_g_now - angle_g_old)/dt;
if rate_g > pi
    rate_g = rate_g - 2*pi;
elseif rate_g < -pi
    rate_g = rate_g + 2*pi;
end
rate_v = k*rate_g;
angle_v_now = angle_v_old + rate_v * dt;
sc_new(3:4, 1) = v*[cos(angle_v_now); sin(angle_v_now)];
sc_new(1:2, 1) = state_c(1:2, 1) + sc_new(3:4, 1)*dt;
angle_v_old = angle_v_now;
angle_g_old = angle_g_now;

end

