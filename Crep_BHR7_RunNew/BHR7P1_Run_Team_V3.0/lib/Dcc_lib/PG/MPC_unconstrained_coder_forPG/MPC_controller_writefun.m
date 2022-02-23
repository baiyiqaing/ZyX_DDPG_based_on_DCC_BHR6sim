% this is a test project
% a output mpc controller 
% 20200130 home
% 20201215 bit write fun CONTROL_T means MPC_T!!!
% 20210427 bit for dcc pg frame

function  [Mu_out, Mc_out, A, B, C] = MPC_controller_writefun(Ac, Bc, Cc, dc, CONTROL_T, T_pre, R, tips_plag, writefun_flag, controllerName, refTraName)
% A = [0 1; 1 0.2]; B = [0; 1]; C = [1, 1]; CONTROL_T = 0.004; T_pre = 1.5; R = 1e-3; % test

[A, B] = c2d(Ac, Bc, CONTROL_T);
C = Cc;
N = T_pre/CONTROL_T;
[h, ~] = size(A);

Mx = []; % cal Mx
M_temp = eye(h);
for i = 1:N
    
    M_temp = M_temp * A;
    Mx = [Mx; C*M_temp];
    
end

Mu = C * B; % cal Mu
M_temp = eye(h);
for i = 1:N - 1 % cal the first list of Mu
    
        M_temp = M_temp * A;
        Mu = [Mu; C * M_temp * B];
        
end
Mu(1) = Mu(1) + 1 * dc;
M_temp = Mu;
s = size(C * B);
for i = 2:N
    
    M_temp = [zeros(s); M_temp(1:end - s(1), :)];
    Mu = [Mu, M_temp];
    
end

n = size(Mu);
E = zeros(1, n(2));
E(1) = 1;
Mu_out = E * ((Mu' * Mu + R * eye(n)) \ Mu');
Mc_out = Mx; 

if tips_plag == 1
    
    disp('U_k = Mu_out * (Y_k^{ref} - Mc_out * X_k)');
    disp('Ac Bc Cc present a continuous system, return A B C present a discrete system.');
    disp('Bigger T_pre with Smaller R can get a better tracking.');
    
end

if writefun_flag == 1
    
    % .c
    fid = fopen([controllerName, '.c'], 'w');
   
    n_ = N - 1;
    fprintf(fid, '//This function requires dcc_con_base.h \n');
    fprintf(fid, '#include "..\\..\\Base\\dcc_con_base.h"\n');
    fprintf(fid, ['#include "', controllerName, '.h" \n\n']);
    % required for simulation in matlab-vrep
%     fprintf(fid, '#include "..\\..\\Base\\dcc_Mat.h"\n#include "..\\..\\Base\\dcc_Mat.c"\n\n');
    
    fprintf(fid, 'const double dVeMu_1x%d[1][nNumPre] = {', N);
    for i = 1:N-1
        fprintf(fid, '%12.16f,\n', Mu_out(i));
    end
    fprintf(fid, '%12.16f};\n', Mu_out(N));
    
    [~, l_]= size(Mc_out);
    fprintf(fid, '\nconst double dMaMx_%dx%d[nNumPre][nStateNum] = {', N, l_);
    for i = 1:N-1
        fprintf(fid, '{');
        for j = 1:l_-1
            fprintf(fid, '%12.16f,', Mc_out(i, j));
        end
        fprintf(fid, '%12.16f},\n', Mc_out(i, l_));
    end
    fprintf(fid, '{');
    for j = 1:l_-1
        fprintf(fid, '%12.16f,', Mc_out(n_, j));
    end
    fprintf(fid, '%12.16f}};', Mc_out(n_, l_));
    fprintf(fid, '\nconst double dMPC_T = %f;', CONTROL_T);
    fprintf(fid, ['\ndouble d', controllerName, 'Conval[2];\n']);
    
    fprintf(fid, '\n/**');
    fprintf(fid, '\nRef trajectorys in x direction & y direction are required');
    fprintf(fid, '\nTo obtain a good control performance, States should be calcualted from an observer');
    fprintf(fid, '\nTo use as an tracking controller, States can calculate from Val^{sens} - Val^{ref}, and Ref trajectorys can be calculated from Tra^{ref} - Tra^{sens}, Tra^{sens} is expanded to the same dimension as Tra^{ref} but value is maintaining the same');
    fprintf(fid, '\n*/');
    fprintf(fid, ['\nvoid fnv', controllerName, 'CalConval(double dVe', refTraName, 'Refx_%dx1[nNumPre][1], double dVe', refTraName, 'Refy_%dx1[nNumPre][1], double dVeStatex_%dx1[nStateNum][1], double dVeStatey_%dx1[nStateNum][1]) {'], N, N, l_, l_);
    fprintf(fid, ['\n\tdouble dVe', refTraName, 'Relx_%dx1[nNumPre][1], dVe', refTraName, 'Rely_%dx1[nNumPre][1];'], N, N);
    fprintf(fid, ['\n\tdcc_fnvMatMet(&dMaMx_%dx%d[0][0], &dVeStatex_%dx1[0][0], nNumPre, nStateNum, 1, ''*'', &dVe', refTraName, 'Relx_%dx1[0][0]);'], N, l_, l_, N);
    fprintf(fid, ['\n\tdcc_fnvMatMet(&dMaMx_%dx%d[0][0], &dVeStatey_%dx1[0][0], nNumPre, nStateNum, 1, ''*'', &dVe', refTraName, 'Rely_%dx1[0][0]);'], N, l_, l_, N);
    fprintf(fid, ['\n\tdouble dVeDelta', refTraName, 'x_%dx1[nNumPre][1], dVeDelta', refTraName, 'y_%dx1[nNumPre][1];'], N, N);
    fprintf(fid, ['\n\tdcc_fnvMatMet(&dVe', refTraName, 'Refx_%dx1[0][0], &dVe', refTraName, 'Relx_%dx1[0][0], nNumPre, 1, 1, ''-'', &dVeDelta', refTraName, 'x_%dx1[0][0]);'], N, N, N);
    fprintf(fid, ['\n\tdcc_fnvMatMet(&dVe', refTraName, 'Refy_%dx1[0][0], &dVe', refTraName, 'Rely_%dx1[0][0], nNumPre, 1, 1, ''-'', &dVeDelta', refTraName, 'y_%dx1[0][0]);'], N, N, N);
    fprintf(fid, '\n\tdouble dGaUx[1][1], dGaUy[1][1];');
    fprintf(fid, ['\n\tdcc_fnvMatMet(&dVeMu_1x%d[0][0], &dVeDelta', refTraName, 'x_%dx1[0][0], 1, nNumPre, 1, ''*'', &dGaUx[0][0]);'], N, N); 
    fprintf(fid, ['\n\tdcc_fnvMatMet(&dVeMu_1x%d[0][0], &dVeDelta', refTraName, 'y_%dx1[0][0], 1, nNumPre, 1, ''*'', &dGaUy[0][0]);\n'], N, N); 
    
    fprintf(fid, ['\n\td', controllerName, 'Conval[0] = dGaUx[0][0];']);
    fprintf(fid, ['\n\td', controllerName, 'Conval[1] = dGaUy[0][0];\n']);
    
    fprintf(fid, '\n}\n\n');
    
    fclose(fid);
    
    % .h
    fid = fopen([controllerName, '.h'], 'w');
    fprintf(fid, '#pragma once \n');
    
    fprintf(fid, '\n#define nNumPre %d\n#define nStateNum %d\n', N, l_);
    
    fprintf(fid, ['\nextern double d', controllerName, 'Conval[2];\n']);
    
    fprintf(fid, ['\nvoid fnv', controllerName, 'CalConval(double dVe', refTraName, 'Refx_%dx1[nNumPre][1], double dVe', refTraName, 'Refy_%dx1[nNumPre][1], double dVeStatex_%dx1[nStateNum][1], double dVeStatey_%dx1[nStateNum][1]);\n'], N, N, l_, l_);
    
    fclose(fid);
end



end


