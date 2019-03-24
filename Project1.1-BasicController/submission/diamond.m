function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

%======Define points must pass======
points_must = [0, 0, 0;
              0.25, 2^(1/2), 2^(1/2);
              0.5, 0, 2*2^(1/2);
              0.75, -2^(1/2), 2^(1/2);
              1, 0, 0];
t_total = 8;
%======Calculate output======
if t<t_total/4
   [pos, vel, acc] = traject_interpolation(0, t_total/4, t, points_must(1,:), points_must(2,:));
elseif t>t_total/4 && t<t_total/2
   [pos, vel, acc] = traject_interpolation(t_total/4, t_total/2, t, points_must(2,:), points_must(3,:));
elseif t>t_total/2 && t<3 * t_total/4
   [pos, vel, acc] = traject_interpolation(t_total/2, 3 * t_total/4, t, points_must(3,:), points_must(4,:));
elseif t>3 * t_total/4 && t<t_total
   [pos, vel, acc] = traject_interpolation(3 * t_total/4, t_total, t, points_must(4,:), points_must(5,:)); 
else
    pos = [1; 0; 0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
end
    yaw = 0;
    yawdot = 0;

function [pos, vel, acc] = traject_interpolation(t_start, t_finish, t, point_start, point_final)
    pos = zeros(1,3); vel = zeros(1,3); acc = zeros(1,3);
    matrix_Quintic = [1, t_start, t_start^2, t_start^3, t_start^4, t_start^5;
                      0, 1, 2*t_start, 3*t_start^2, 4*t_start^3, 5*t_start^4;
                      0, 0, 2, 6*t_start, 12*t_start^2, 20*t_start^3;
                      1, t_finish, t_finish^2, t_finish^3, t_finish^4, t_finish^5;
                      0, 1, 2*t_finish, 3*t_finish^2, 4*t_finish^3, 5*t_finish^4;
                      0, 0, 2, 6*t_finish, 12*t_finish^2, 20*t_finish^3];
    matrix_Time = [1, t, t^2, t^3, t^4, t^5;
                   0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4;
                   0, 0, 2, 6*t, 12*t^2, 20*t^3;];
    matrix_State = [point_start; zeros(2,3); point_final; zeros(2,3)];
    matrix_Coefficient = matrix_Quintic \ matrix_State;
    matrix_Result = matrix_Time * matrix_Coefficient;
    pos = matrix_Result(1,:); vel = matrix_Result(2,:); acc = matrix_Result(3,:);
end
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
