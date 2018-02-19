% test freq_sweep.m
% This File tests freq_sweep.m


% These can be streamlined by stitching all of the waves together. They
% all take the same number of samples. Just insert a series of zeros with
% enough to know that distance is not an issue (between segments). split
% using the reference signal that will have no noise.
start_f = 200;
end_f = 20000;
f_points = 50;
dur_tone_s = (1/start_f) * 5;  % smallest frequency and n waves of it

fs = 81000;
dur_start_s = 0.2;

n = 9;
% output_f = @filter_tone;
output_f = @mydaq_tone;
loudness = 0.1;
distances = [1, 4, 7, 10, 13, 16, 19];
len_d = length(distances);

amps_m = zeros(len_d, f_points);
amps_std = zeros(len_d, f_points);
phs_m = zeros(len_d, f_points);
phs_std = zeros(len_d, f_points);

for i = 1:len_d
   ['Press key when ready for distance ' num2str(distances(i)) ' m']
   pause
   [amp_m_db, amp_std_db, ph_m, ph_std, all_freq] = f_sweep_system(start_f, end_f, f_points, dur_tone_s, fs, n, loudness, output_f);
   amps_m(i, :) = amp_m_db;
   amps_std(i, :) = amp_std_db;
   phs_m(i, :) = ph_m;
   phs_std(i, :) = ph_std;
   
end

figure(1)
legend_str = zeros(1, len_d);
for i = 1:len_d
%     legend_str(i) = [num2str(distances(i)) ' Meters'];
    
    subplot(2,1,1)
%     errorbar(all_freq, amp_m_db, amp_std_db);
    errorbar(all_freq, amps_m(i, :), amps_std(i, :));
    xlabel('Frequency (Hz)');
    ylabel('Amplitude');
    title('System Amplitude Response')
    hold on
    subplot(2,1,2)
    errorbar(all_freq, phs_m(i, :), phs_std(i, :));
    xlabel('Frequency (Hz)');
    ylabel('Phase (Degrees');
    title('System Phase Response');
    hold on;
end
subplot(2,1,1)
legend('1 m', '4 m', '7 m', '10 m', '13 m', '15 m');
subplot(2,1,2)
legend('1 m', '4 m', '7 m', '10 m', '13 m', '15 m');
