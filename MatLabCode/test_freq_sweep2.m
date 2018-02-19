% test freq_sweep.m
% This File tests freq_sweep.m


% These can be streamlined by stitching all of the waves together. They
% all take the same number of samples. Just insert a series of zeros with
% enough to know that distance is not an issue (between segments). split
% using the reference signal that will have no noise.
start_f = 200;
end_f = 20000;
f_points = 100;
dur_tone_s = (1/start_f) * 18;  % smallest frequency and n waves of it
loudness = 9;

fs = 2^16; %end_f * 2.5;
dur_start_s = 0.2;
n = 5;
% output_f = @filter_tone;
output_f = @mydaq_tone;

% should work
[amp_m_db, amp_std_db, ph_m, ph_std, all_freq] = f_sweep_system(start_f, end_f, f_points, dur_tone_s, fs, n, loudness, output_f);

figure(1)
% legend_str = zeros(1, len_d);
% for i = 1:len_d
%     legend_str(i) = [num2str(distances(i)) ' Meters'];
    
subplot(2,1,1)
errorbar(all_freq, amp_m_db, amp_std_db);
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('System Amplitude Response')

subplot(2,1,2)
errorbar(all_freq, ph_m, ph_std);
xlabel('Frequency (Hz)');
ylabel('Phase (Degrees');
title('System Phase Response');
% hold on;


