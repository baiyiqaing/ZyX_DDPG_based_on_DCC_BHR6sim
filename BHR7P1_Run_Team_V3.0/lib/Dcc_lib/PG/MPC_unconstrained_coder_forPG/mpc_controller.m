% this is a test project
% a output mpc controller 
% 20200130 home

function  [Mu_out, Mc_out, A, B, C] = mpc_controller(Ac, Bc, Cc, dc, CONTROL_T, T_pre, R, tips_plag)
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



end


