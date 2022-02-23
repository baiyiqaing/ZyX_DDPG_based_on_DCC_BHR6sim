% this is a test project 
% to cal com_x and com_y from zmp
% 20200131 home
% 20200729 bit QP
% 20201201 bit dcom
% 20201210 bit zmp_cal stepadj

function [com, dcom, zmp_cal] = cal_com(zmp, L_walk, T_pre, R, stepadj_p, CONTROL_T, zc, g, solver, plot_flag, tips_plag)
% zc = 0.8; g = 9.8; CONTROL_T = 0.004; T_pre = 1.5; R = 1e-4; plot_flag = 1; tips_plag = 0; N_step = 6; T_step = 0.6; L_step = 0.3;  P_sin = 0.8; L_sin = 0.1; zmp_width = 0.16; [zmp, t, L_walk] = cal_zmp(N_step, T_step, P_sin, L_step, L_sin, T_pre, zmp_width, CONTROL_T, 0); % test

zmp_x = zmp(1, :);
zmp_y = zmp(2, :);

Ac = [0 1 0; 0 0 1; 0 0 0];
Bc = [0; 0; 1];
Cc = [1 0 -zc/g];
dc = 0;

if solver == 'A'
    [Mu_out, Mc_out, A, B ,C] = mpc_controller(Ac, Bc, Cc, dc, CONTROL_T, T_pre, R, tips_plag);
elseif solver == 'Q'
    Yx_ref = zmp_x;
    Yy_ref = zmp_y;
end
    
[~, N_all] = size(zmp);
N = T_pre/CONTROL_T;
com_x = stepadj_p * ones(1, N_all);
com_y = L_walk * ones(1, N_all);
dcom_x = zeros(1, N_all);
dcom_y = zeros(1, N_all);
zmp_x_cal = zeros(1, N_all);
zmp_y_cal = L_walk * ones(1, N_all);
x = [0; 0; 0]; % state of LIPM in x direction
y = [0; 0; 0]; % state of LIPM in y direction

if solver == 'A' % analize
    for i = 1:N_all - N

        % get Umpc
        Px_ref = zmp_x(i:i + N - 1)'; % take ref from now turn out to be better
        Py_ref = zmp_y(i:i + N - 1)'; % that is from i not i + 1
        U_x = Mu_out * (Px_ref - Mc_out * x);
        U_y = Mu_out * (Py_ref - Mc_out * y);
        % get Umpc

        % refresh
        x = A * x + B * U_x;
        y = A * y + B * U_y;
        Px_cal = C * x;
        Py_cal = C * y;
        % refresh

        com_x(i) = x(1);
        com_y(i) = y(1);
        dcom_x(i) = x(2);
        dcom_y(i) = y(2);
        zmp_x_cal(i) = Px_cal;
        zmp_y_cal(i) = Py_cal;

    end
elseif solver == 'Q' % QP
    for i = 1:N_all - N
        
        optsQP = optimoptions('quadprog','Display','off');
        % get Umpc
        [Hx, gx, A, B, C] = mpc_cal_Hg(Ac, Bc, Cc, dc, CONTROL_T, T_pre, R, x, Yx_ref, i, 0);
        [Hy, gy, ~, ~, ~] = mpc_cal_Hg(Ac, Bc, Cc, dc, CONTROL_T, T_pre, R, y, Yy_ref, i, 0);
        U_x = quadprog(Hx, gx, [], [], [], [], [], [], [], optsQP);
        U_y = quadprog(Hy, gy, [], [], [], [], [], [], [], optsQP);
        % get Umpc
        
        % refresh
        x = A * x + B * U_x(1);
        y = A * y + B * U_y(1);
        Px_cal = C * x;
        Py_cal = C * y;
        % refresh
        
        com_x(i) = x(1);
        com_y(i) = y(1);
        dcom_x(i) = x(2);
        dcom_y(i) = y(2);
        zmp_x_cal(i) = Px_cal;
        zmp_y_cal(i) = Py_cal;
        
    end   
end
    

com = [com_x; com_y];
dcom = [dcom_x; dcom_y];
zmp_cal = [zmp_x_cal; zmp_y_cal];

if plot_flag == 1
    
    figure; plot(com_x); hold on; plot(zmp_x); plot(zmp_x_cal, '.-'); hold off;
    title('pg x');
    figure; plot(com_y); hold on; plot(zmp_y); plot(zmp_y_cal, '.-'); hold off;
    title('pg y');
    
end



end
    
    
    
    
    
    
    
    
    
    
    
    