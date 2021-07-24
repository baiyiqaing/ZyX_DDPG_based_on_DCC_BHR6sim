% this is a test project
% to generate zmp_x and zmp_y
% 20200130 home
% 20201201 bit: add supsignal 
% 20201209 bit: stepadj

function [zmp, t, L_walk, PROGRAM_N, supsignal, suppoly] = cal_zmp(N_step, T_step, P_sin, L_step, L_sin, T_pre, zmp_width, stepadj_n, stepadj_p, CONTROL_T, method, plot_flag)
% N_step = 6; T_step = 0.6; P_sin = 0.8; L_step = 0.3; L_sin = 0.1; T_pre = 1.5; zmp_width = 0.16; CONTROL_T = 0.004; % test

speed = L_step/T_step*3.6;
T_ready = 1.5;
T_end = 1.5;
T_walk = N_step * T_step;
T_sin = P_sin * T_step;
T_dou =  (1 - P_sin) * T_step;
L_walk = (N_step - 1) * L_step;
y_zmp_mark = zeros(1, 2 * N_step);
x_zmp_mark = zeros(1, 2 * N_step);
t_zmp_mark = zeros(1, 2 * N_step);

disp([num2str(speed), 'km/h']);

adj_temp = 0.0;
for i = 1:N_step

    y_zmp_mark(2 * i - 1) = (i - 1) * L_step; % last step end, sin start
    y_zmp_mark(2 * i) = (i - 1) * L_step + L_sin; % sin end, dou start
    
    if  i == N_step
        
        y_zmp_mark(2 * i) = (i - 1) * L_step; % last step is a follow step
        
    end
    
    if i >= stepadj_n
        adj_temp = stepadj_p;
    end
        
    x_zmp_mark(2 * i - 1) = - 0.5 * (- 1) ^ i  * zmp_width + adj_temp; % last step end, sin start
    x_zmp_mark(2 * i) = - 0.5 * (- 1) ^ i  * zmp_width + adj_temp; % sin end, dou start
    
    t_zmp_mark(2 * i - 1) = (i - 1) * T_step; % last step end, sin start
    t_zmp_mark(2 * i ) = (i - 1) * T_step + T_sin; % sin end, dou start
    
end

y_zmp_mark =[0.0, 0.0, y_zmp_mark, (N_step - 1) * L_step, (N_step - 1) * L_step]; % the last step is a follow step
x_zmp_mark =[0.0, 0.0, x_zmp_mark, stepadj_p, stepadj_p];
t_zmp_mark = t_zmp_mark + T_ready + T_dou;
t_zmp_mark = [0.0, T_ready, t_zmp_mark, T_walk + T_ready + T_dou, T_walk + T_ready +  T_dou + T_end + T_pre];

t = 0:CONTROL_T:T_walk + T_ready +  T_dou + T_end + T_pre;

if method == 'P'
    y_zmp = interp1(t_zmp_mark, y_zmp_mark, t, 'pchip');
    x_zmp = interp1(t_zmp_mark, x_zmp_mark, t, 'pchip');
elseif method == 'L'
    y_zmp = interp1(t_zmp_mark, y_zmp_mark, t, 'linear');
    x_zmp = interp1(t_zmp_mark, x_zmp_mark, t, 'linear');
elseif method == 'N'
    y_zmp = interp1(t_zmp_mark, y_zmp_mark, t, 'nearest');
    x_zmp = interp1(t_zmp_mark, x_zmp_mark, t, 'nearest');
end

[~, l] = size(x_zmp);
y_ankle = 0;
y_ankle_next = L_step;
supsignal = zeros(1, l);
suppoly_x = zeros(2, l);
suppoly_y = zeros(2, l);

adj_temp_l = 0.0;
adj_temp_r = 0.0;
for i = 1:l
    
    if (i * CONTROL_T - T_ready - T_dou - 0.0001) / T_step + 1 >= stepadj_n
        adj_temp_r = stepadj_p;
    end
    if (i * CONTROL_T - T_ready - 0.0001) / T_step + 1 >= stepadj_n
        adj_temp_l = stepadj_p;
    end
    
    if x_zmp(i) == -0.5 * zmp_width + adj_temp_l % l sup
        supsignal(i) = -1;
        suppoly_x(1, i) = -0.5 * zmp_width + 0.05 + adj_temp_l;
        suppoly_x(2, i) = -0.5 * zmp_width - 0.07 + adj_temp_l;
        suppoly_y(1, i) = y_ankle + 0.15;
        suppoly_y(2, i) = y_ankle - 0.09;
    elseif x_zmp(i) == 0.5 * zmp_width + adj_temp_r % r sup
        supsignal(i) = 1;
        suppoly_x(1, i) = 0.5 * zmp_width + 0.07 + adj_temp_r;
        suppoly_x(2, i) = 0.5 * zmp_width - 0.05 + adj_temp_r;
        suppoly_y(1, i) = y_ankle + 0.15;
        suppoly_y(2, i) = y_ankle - 0.09;
    else % d sup
        supsignal(i) = 0;
        suppoly_x(1, i) = 0.5 * zmp_width + 0.07 + adj_temp_r;
        suppoly_x(2, i) = -0.5 * zmp_width - 0.07 + adj_temp_l;
        if i * CONTROL_T <= T_ready + T_dou || i * CONTROL_T >= T_walk 
            suppoly_y(1, i) = y_ankle + 0.15;
            suppoly_y(2, i) = y_ankle - 0.09;
        else
            suppoly_y(1, i) = y_ankle_next + 0.15;
            suppoly_y(2, i) = y_ankle - 0.09;
        end
            
    end
    
    if y_zmp(i) >= y_ankle_next - L_sin
        y_ankle = y_ankle_next;
    end
    if y_ankle == y_ankle_next && supsignal(i) ~= 0
        y_ankle_next = y_ankle_next + L_step;
    end
    
end
suppoly = [suppoly_x; suppoly_y];

if plot_flag == 1
    
    figure; plot(t, x_zmp); hold on; plot(t, supsignal); hold off;
    title('pg zmp x');
    figure; plot(t, y_zmp); hold on; plot(t, supsignal); hold off;
    title('pg zmp y');
end

zmp = [x_zmp; y_zmp];
[~, PROGRAM_N] = size(zmp);


end

