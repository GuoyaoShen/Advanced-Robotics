function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

%define the circle
r = 5;
h_helix = 2.5;
o = [0;0;0];
circle = 2 * pi * r;
t_total = 9;  %total time to finish
%define angular velocity
ang_velo = 2*pi/t_total;
%calculate output
[z_pos, z_vel, z_acc] = traject_interpolation(0, t_total, t, o(3), h_helix);  %give out z axis state
[circle_pos, circle_vel, circle_acc] = traject_interpolation(0, t_total, t, 0, circle);  %give out the line state on circle
angle = 2 * pi * circle_pos/circle;  %calculate the angle passed

if t < t_total
    %calcuate direction vector of pos, vel, acc on circle
    dir_pos = [r * cos(angle); r * sin(angle)];
    dir_pos = dir_pos/norm(dir_pos);
    dir_vel = [-r * ang_velo * sin(angle); r * ang_velo * cos(angle)];
    dir_vel = dir_vel/norm(dir_vel);
    dir_acc = [-r * ang_velo^2 * cos(angle); -r * ang_velo^2 * sin(angle)];
    dir_acc = dir_acc/norm(dir_acc);
    %calculate pos, vel, acc
    pos = [r * dir_pos + [o(1); o(2)]; z_pos + o(3)];
    vel = [circle_vel * dir_vel; z_vel];
    acc = [circle_acc * dir_vel + (circle_vel^2/r) * dir_acc; z_acc];
    yaw = (pi/2) + ang_velo * t;
    yawdot = ang_velo;
else
    pos = [o(1) + r;o(2);o(3) + h_helix];
    vel = [0;0;0];
    acc = [0;0;0];
    yaw = (pi/2) + ang_velo * t_total;
    yawdot = 0;
end

%interpolation function
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
    matrix_State = [point_start; zeros(2,1); point_final; zeros(2,1)];
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
