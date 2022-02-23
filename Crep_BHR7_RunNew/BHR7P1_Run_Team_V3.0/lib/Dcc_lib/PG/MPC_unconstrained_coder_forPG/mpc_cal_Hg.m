% this is a test project
% a output mpc controller 
% 20200130 home
% 20200729 bit changed for QP

function  [H, g, A, B, C] = mpc_cal_Hg(Ac, Bc, Cc, dc, CONTROL_T, T_pre, R, xk, Y_ref, k, tips_plag)
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

len = length(Y_ref);
if k + N > len
    Y_ref_temp = Y_ref(end) * ones(1, k + N - length(Y_ref));
    Yk_ref = [Y_ref(k + 1:end), Y_ref_temp];
else
    Yk_ref = Y_ref(k + 1:k + N);
end
H = Mu' * Mu + R * eye(N);
g = 2 * (xk' * Mx' - Yk_ref) * Mu;


if tips_plag == 1
    
    disp('U_k = Mu_out * (Y_k^{ref} - Mc_out * X_k)');
    disp('Ac Bc Cc present a continuous system, return A B C present a discrete system.');
    disp('Bigger T_pre with Smaller R can get a better tracking.');
    
end



end


