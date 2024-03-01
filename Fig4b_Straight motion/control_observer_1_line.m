function  sc_new= control_observer_1_line(ao, state_c,dt)
% observer straight motion with constant acceleration
% start from (0,1) to (0,9), the acceleration is 2m/s^2


y_now = state_c(2, 1);
vy_now = state_c(4, 1);

if vy_now > 0
    flag = 1;
else
    flag = -1;
end

if flag  % forward
    if y_now >= 5
        ay = -ao;
    else
        ay = ao;
    end 
else     % backward
    if y_now <= 5
        ay = ao;
    else
        ay = -ao;
    end 
end

sc_new = zeros(4, 1);
sc_new(1:2, 1) = state_c(1:2, 1) + state_c(3:4, 1)*dt;
sc_new(3:4, 1) = state_c(3:4, 1) + dt*[0; ay];


end

