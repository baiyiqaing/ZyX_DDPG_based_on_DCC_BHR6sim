data = load('DCC_C-rep.dat');

del_cx = data(:, 97); del_vx = data(:, 98); del_cy = data(:, 99); del_vy = data(:, 100);

lipm_x = data(:, 101); zrel_x = data(:, 102); zref_x = data(:, 103); zpg_x = data(:, 104);
lipm_y = data(:, 105); zrel_y = data(:, 106); zref_y = data(:, 107); zpg_y = data(:, 108);

tpc_x = data(:, 110); tpc_y = data(:, 111);

frz_rel = data(:, 122); frz_ref = data(:, 123); frz_ = data(:, 124);
trx_rel = data(:, 125); trx_ref = data(:, 126); trx_ = data(:, 127);
try_rel = data(:, 128); try_ref = data(:, 129); try_ = data(:, 130);
flz_rel = data(:, 131); flz_ref = data(:, 132); flz_ = data(:, 133);
tlx_rel = data(:, 134); tlx_ref = data(:, 135); tlx_ = data(:, 136);
tly_rel = data(:, 137); tly_ref = data(:, 138); tly_ = data(:, 139);

forw = data(:, 154); back = data(:, 155); left = data(:, 156); righ = data(:, 157);
forw_w = data(:, 154 + 4); back_w = data(:, 155 + 4); left_w = data(:, 156 + 4); righ_w = data(:, 157 + 4);

supsig = data(:, 152);
% plot(supsig);
for i = 1:length(supsig)
   if supsig(i) == 1 
       supsig(i) = -1; 
   elseif supsig(i) == 2 
       supsig(i) = 1; 
   else
       supsig(i) = 0;
   end
end

%% del com
figure; 
subplot(211); plot(-del_cx); hold on; plot(-del_vx); plot(-lipm_x); plot(-tpc_x); hold off;
legend('del_cx', 'del_vx', 'lipm_x', 'tpc_x');
subplot(212); plot(-zrel_x); hold on; plot(-zref_x); plot(-zpg_x); plot(0.05 * supsig); hold off; 
legend('zrel_x', 'zref_x', 'zpg_x', 'supsig');
figure; 
subplot(211); plot(del_cy); hold on; plot(del_vy); plot(lipm_y); plot(tpc_y); hold off;
legend('del_cy', 'del_vy', 'lipm_y', 'tpc_y');
subplot(212); plot(zrel_y); hold on; plot(zref_y); plot(zpg_y);  plot(0.05 * supsig); hold off;
legend('zrel_y', 'zref_y', 'zpg_y', 'supsig');
%% footft
figure; plot(frz_rel); hold on; plot(frz_ref); hold off;
legend('frz_rel', 'frz_ref');
figure; plot(flz_rel); hold on; plot(flz_ref); hold off;
legend('flz_rel', 'flz_ref');
%% lipm
figure;
plot(zrel_x, '.-b'); hold on; plot(left, 'k', 'linewidth', 2); plot(righ, 'k', 'linewidth', 2); plot(zref_x, '-r', 'linewidth', 1.5); hold off;
legend('z_{rel}', '-', '-', 'z_{ref}');
figure;
plot(zrel_y, '.-b'); hold on; plot(forw, 'k', 'linewidth', 2); plot(back, 'k', 'linewidth', 2); plot(zref_y, '-r', 'linewidth', 1.5); hold off;
legend('z_{rel}', '-', '-', 'z_{ref}');
%% suppoly
figure;
plot(forw_w, 'k'); hold on; plot(back_w, 'k');
figure;
plot(left_w, 'k'); hold on; plot(righ_w, 'k');
figure;
plot(supsig);
