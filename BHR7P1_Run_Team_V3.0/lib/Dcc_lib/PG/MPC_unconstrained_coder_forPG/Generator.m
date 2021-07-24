% to generate TPCMPC code
% 20201215 bit dcc
% 20210427 bit for dcc pg frame
% 20210520 bit bug

% paras for the generator
CONTROL_T = 0.004;
zc = 0.66;
g = 9.8;
Ac = [0 1 0; 0 0 1; 0 0 0];
Bc = [0; 0; 1];
Cc = [1 0 -zc/g];
dc = 0;

% pattern generate test
N_step = 10;
T_step = 0.8;
L_step = 0.2;
zmp_width = 0.16;
P_sin = 0.9;
L_sin = 0.05;
T_pre = 2.0;
zmp_spline_method = 'P';
stepadj_n = 6; 
stepadj_p = 0.05;
com_mpc_solver = 'A';
com_uR = 1e-7;

% con flags
zmpplot_flag = 0;
comtips_plag = 0;
complot_flag = 1;

% spline for zmp
[zmp_ref, t, L_walk, PROGRAM_N, supsignal, suppoly] = cal_zmp(N_step, T_step, P_sin, L_step, L_sin, T_pre, zmp_width, stepadj_n, stepadj_p, CONTROL_T, zmp_spline_method, zmpplot_flag);
% analytical mpc for com
[com_ref, dcom_ref, zmp_cal] = cal_com(zmp_ref, L_walk, T_pre, com_uR, stepadj_p, CONTROL_T, zc, g, com_mpc_solver, complot_flag, comtips_plag);

[Mu_out, Mc_out, A, B, C] = MPC_controller_writefun(Ac, Bc, Cc, dc, CONTROL_T, T_pre, com_uR, 1, 1, 'PGMPC', 'Zmp');
