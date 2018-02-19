% Goal: encode the lfsr code into a waveform that can be transmitted.
% Determine the bandwidth use for generating this signal

fs = 80000;
% fs = 200000;

chipping_f = 10000; % The bandwidth is half the chip_freq  % 2.5k = 1.25k bd, 5k = 2.5k bd, 10k = 5k bd, 
lfsr_code = [1,0,0,1,1,0,1,0,0,1,0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,1,1,1,1]; % created with s=[1, 1, 0, 0, 1], t= [5, 2] using LFSR(s, t) 
lfsr_code = [lfsr_code lfsr_code];

% center on 10k, vary width

bw = chipping_f/2;
mid = 10000;
a_multi = (mid - 0.5*bw)/bw;


filter_len = round(length(lfsr_code) * 0.5 * a_multi);  % 0.5 = 1 signal width
samp_per_c = round(fs/chipping_f);
% lfsr_code = [1,1,1,0,0,0,1,1,1];
% lfsr_code = [1, 1, -1];
% lfsr_code = [lfsr_code lfsr_code lfsr_code];
% lfsr_code = [lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code];
% lfsr_code = [lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code lfsr_code];

% [new_signal, new_fft, center_wave, center_wave_fft] = modulate_signal(lfsr_code, center_f, fs, chipping_f, filter_fcx);

[new_signal, new_fft] = modulate_signal(lfsr_code, samp_per_c, filter_len, fs);
[H, F] = f_analysis(new_signal, fs);
len_h = length(H);
portion_to_disp = 0.6;
max_idx = round(portion_to_disp*len_h);
plot(F(1:max_idx), abs(H(1:max_idx)));
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('New Signal Frequency Representation');% figure(1)
% subplot(1,2,1)
% plot(new_signal)
% subplot(1,2,2)
% plot(abs(new_fft))


% plan on 2 second signal
sig_time = 2;  % seconds
signal_loc = 0.5;  % This places the wanted signal in the center.
total_points = fs * sig_time;
signal_idx = round(total_points*signal_loc);
x = zeros(1, signal_idx - 1);
y = new_signal;
z = zeros(1, total_points - (signal_idx - 1 + length(new_signal)));
a = length(x) + length(y) + length(z);
input_signal = [x y z];

% plot(input_signal);


% captured_data = input_signal;
% time = 0:1/fs:(sig_time-1/fs);
[captured_data, time] = run_mydaq(fs, input_signal);  %  s.startForeground();




% plot signal that was sent
figure(1)
subplot(2,1,1)
plot(time, input_signal);
xlabel('Time [s]');
ylabel('Magnitude');
title('New Signal Time Representation');

subplot(2,1,2);
[H, F] = f_analysis(input_signal, fs);
len_h = length(H);
portion_to_disp = 0.6;
max_idx = round(portion_to_disp*len_h);
plot(F(1:max_idx), abs(H(1:max_idx)));
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('New Signal Frequency Representation');

% plot signal that was received
figure(2)
subplot(2,1,1)
plot(time, captured_data);
ylabel('Magnitude');
xlabel('Time');
title('Acquired Signal Time Represenation');

subplot(2,1,2)
[H, F] = f_analysis(captured_data, fs);
len_h = length(H);
portion_to_disp = 0.4;
max_idx = round(portion_to_disp*len_h);
plot(F(1:max_idx), abs(H(1:max_idx)));
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('Acquired Signal Frequency Representation');


% perform correlation with short method
figure(3)
% % fft_cap = fft(captured_data);
% % correlation with converting to frequency, multiplying, and bringing back
% test_captured_data = [captured_data zeros(1, length(new_fft))];
% test_new_signal = [new_signal zeros(1, length(captured_data))];
% combined_fft = fft(test_captured_data) .* fft(test_new_signal);
% corr_time = ifft(combined_fft);
% plot(corr_time);

% correlation using the long method
% corr_data = zeros(1, length(captured_data) + length(new_fft));
corr_data = zeros(1, length(captured_data));
% captured_data = [captured_data zeros(1, length_new_signal)];
len_n = length(new_signal);
len_c = length(captured_data);
for i = 1:(len_c - len_n - 1)
%     vector_a = captured_data(i:i + len_n - 1);
%     vector_b = new_signal;
%     result_c = vector_a .* vector_b;
%     sum_d = sum(result_c);
    corr_data(i) = sum( captured_data(i:i + len_n - 1) .* new_signal);    
end

plot(time, corr_data);
xlabel('Time (s)');
ylabel('Magnitude');
title('Return Signal Correlation vs Input Signal');
[max_val, idx] = max(corr_data); 
cor_t = time(idx)

% combined = fft_cap .* new_fft;
% corr_data = ifft(combined);
% plot(corr_data);

% 
% figure(2)
% subplot(2,1,1)
% [H, F] = f_analysis(new_signal, fs);
% len_h = length(H);
% portion_to_disp = 0.4;
% max_idx = round(portion_to_disp*len_h);
% plot(F(1:max_idx), abs(H(1:max_idx)));
% xlabel('Frequency [Hz]');
% ylabel('Magnitude');
% title('New Signal Frequency Representation');
% 
% subplot(2,1,2)
% [H, F] = f_analysis(center_wave, fs);
% len_h = length(H);
% max_idx = round(portion_to_disp*len_h);
% plot(F(1:max_idx), abs(H(1:max_idx)));
% xlabel('Frequency [Hz]');
% ylabel('Magnitude');
% title('Reference Signal Frequency Representation');






