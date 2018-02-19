%Signal_producer.m
% This function takes an lfsr code, modulates it with a given bandwidth and
% center frequency, and sends it to a MyDAQ, receives it, and performs
% cross-correlation on it.
% Written by Nephi Smith 18 Oct 2017

% Sampling Frequency. MyDAQ max fs = 200,000 Hz
fs = 80000;  

% Set these three parameters to change the signal
chipping_f = 1000; % The bandwidth is half the chip_freq  % 2.5k = 1.25k bd, 5k = 2.5k bd, 10k = 5k bd,
bw = chipping_f/2;
center_freq = 4000;
% LFSR Code

% lfsr_code = [1 0 1 0 1 0 1 0];
lfsr_code = [1,0,0,1,1,0,1,0,0,1,0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,1,1,1,1]; % created with s=[1, 1, 0, 0, 1], t= [5, 2] using LFSR(s, t)
% lfsr_code = [lfsr_code lfsr_code lfsr_code lfsr_code];
% lfsr_code = [lfsr_code lfsr_code lfsr_code lfsr_code];
% lfsr_code = [lfsr_code lfsr_code lfsr_code lfsr_code];


% Do not modify these parameters
% chipping_f = bw*2;
% a_multi = (center_freq - 0.5*bw)/bw;  % internal multiplier. This centers everything
% 
% filter_len_zeros = round(length(lfsr_code) * 0.5 * a_multi);  % Multiplying by 0.5 gives 1 signal width of zeros. Multiplying by a_multi positions the center where we want it
% samp_per_chip = round(fs/chipping_f);



% Modulate The Signal
new_signal = make_time_signal(bw, center_freq, lfsr_code, fs);
% [new_signal, new_fft] = modulate_signal(lfsr_code, samp_per_chip, filter_len_zeros);


% volume adjustment
loudness = 0.7;
new_signal = new_signal*loudness;


% Time pad the signal so it happens after the receiver starts listening.
% Must be at least 0.1s long. More consistent if longer
sig_time = 1;  % seconds
signal_loc = 0.5;  % Places signal in the center.
input_signal = time_pad_signal(sig_time, signal_loc, fs, new_signal);

% repeat n times. Report last time
n = 1;
results = zeros(1,n);
results_corrected = zeros(1,n);
speed_sound = 340; %m/s
for i = 1:n

    % Run MyDAQ Execution
    [captured_data, time] = run_mydaq(fs, input_signal);  %  s.startForeground();
    
    captured_data(1, :) = [zeros(1, 10000) input_signal(1:70000)];
    captured_data(1, :) = captured_data(1, :) + 0.01 * captured_data(1, :).^2;

    % Perform cross-correlation
    % corr_data0 = corr_brute(new_signal, captured_data);
    corr_data0 = corr_brute(new_signal, captured_data(1,:));
    corr_data1 = corr_brute(new_signal, captured_data(2,:));


    [max_val, idx] = max(corr_data1); 
    time_ref = time(idx);

    [max_val, idx] = max(corr_data0); 
%     time(idx) - time_ref
    results_corrected(i) = time(idx) - time_ref;  % This correction takes the MyDAQ error from +- 29 cm to +- 3.4 cm
    results(i) = time(idx);
end
std_dist = std(results) * speed_sound;  % in meters
std_dist_cor = std(results_corrected) * speed_sound;  % in meters
detected_distance = results_corrected(1) * speed_sound * 100 % cm

% Begin displaying Results

% Plot Input Signal
figure(1)
subplot(2,4,1)
plot(time, input_signal);
xlabel('Time [s]');
ylabel('Magnitude');
title('New Sig Time Rep');

subplot(2,4,5);
[H, F] = f_analysis(input_signal, fs);
len_h = length(H);
portion_to_disp = 0.6;
max_idx = round(portion_to_disp*len_h);
plot(F(1:max_idx), abs(H(1:max_idx)));
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('New Signal Frequ Rep');

% Plot Received Signals ai0 and ai1 in Time and Frequency
% figure(2)
subplot(2,4,2)
plot(time, captured_data(1,:));
ylabel('Magnitude');
xlabel('Time');
title('Acquired Sig Time Rep ai0');

subplot(2,4,3)
plot(time, captured_data(2,:));
ylabel('Magnitude');
xlabel('Time');
title('Acquired Sig Time Rep ai1');

subplot(2,4,6)
[H, F] = f_analysis(captured_data(1,:), fs);
len_h = length(H);
portion_to_disp = 0.4;
max_idx = round(portion_to_disp*len_h);
plot(F(1:max_idx), abs(H(1:max_idx)));
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('Acquired Sig Freq Rep ai0');

subplot(2,4,7)
[H, F] = f_analysis(captured_data(2,:), fs);
len_h = length(H);
portion_to_disp = 0.4;
max_idx = round(portion_to_disp*len_h);
plot(F(1:max_idx), abs(H(1:max_idx)));
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('Acquired Sig Freq Rep ai1');


% Plot Cross-Corrlation Data
% figure(3)
subplot(2,4,4)
plot(time, corr_data0);
xlabel('Time (s)');
ylabel('Magnitude');
title('Return Sig XCorr ai0 vs Input Sig');

subplot(2,4,8)
plot(time, corr_data1);
xlabel('Time (s)');
ylabel('Magnitude');
title('Return Sig XCorr ai1 vs Input Sig');




