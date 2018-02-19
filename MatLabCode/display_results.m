% Display results from each speaker for the system
% 12 16 2017

% distance sweep 
if 1==0
    figure(111)
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
    
    % plot amplitude loss
    figure(112)
    deltas = amps_m(1, :) - amps_m(len_d,:);
    plot(all_freq, deltas);
    xlabel('Frequency (Hz)');
    ylabel('Signal Loss Closest to Furthest (dB)');
    title('Total Signal Loss');
    max_loss = max(deltas)
end



% Precision 
if 1==0
    for k = 1:length(bws)
        bw = bws(k);
        all_center_freq = freqs(k, :);
    %     freqs(k, :) = all_center_freq;

    %     times_m = mean(time_results);
    %     times_std = std(time_results);
    %     sig_str_m(k, :) = mean(signal_strength);
    %     sig_str_std(k, :) = std(signal_strength);
    % 
    %     % convert to distance measurement
    %     s_sound = 340;
    %     dist_m(k, :) = times_m * s_sound;
    %     dist_std(k, :) = times_std * s_sound;

        figure(k)
%         subplot(1,2,1)
        plot(all_center_freq, dist_std(k, :));
%         errorbar(all_center_freq, dist_m(k, :), dist_std(k, :));
        xlabel('Frequency 10 kHz');
        ylabel('Distance (m) - Error below 0 m');
        ylabel('Distance (m) - std error');
        title(['Consistency vs Center Frequency @BW ' num2str(bw/1000) ' kHz'])

%         subplot(1,2,2)
% %         errorbar(all_center_freq, sig_str_m(k, :), sig_str_std(k, :));
%         plot(all_center_freq, sig_str_std(k, :));
%         xlabel('Frequency 10 kHz');
%         ylabel('Signal Strength Normalized to Reference');
%         title(['Signal Strength vs Center Frequency @BW ' num2str(bw/1000) ' kHz'])
    end
end


% f_sweep
if 1==0
    figure(100)
    % legend_str = zeros(1, len_d);
    % for i = 1:len_d
    %     legend_str(i) = [num2str(distances(i)) ' Meters'];

    subplot(2,1,1)
    errorbar(all_freq, amp_m_db, amp_std_db);
    xlabel('Frequency (Hz)');
    ylabel('Amplitude');
    title('System Amplitude Response')

    subplot(2,1,2)
    errorbar(all_freq, rad2deg(unwrap(deg2rad(ph_m))), ph_std);
    xlabel('Frequency (Hz)');
    ylabel('Phase (Degrees');
    title('System Phase Response');
    % hold on;
end


% filter results
if 1 ==0
    dur = 0.01;  % seconds
    fs = 2^15;% 40000;

    freq = 900;
    n = 1:round(dur*fs);
    % input_signal = sin(2 * pi * freq * n / fs);
    % input_signal = input_signal + sin(2 * pi * 15000 * n / fs);
    % input_signal = input_signal + sin(2 * pi * 8000 * n / fs);
    input_signal = make_tone(freq, dur, fs);

    % [H, F] = f_analysis(points(length(points)/100000:length(points)/9), fs);
    [H, F] = f_analysis(points, fs);
    len_h = length(H);
    portion_to_disp = 0.8;
    max_idx = round(portion_to_disp*len_h);
    plot(F(1:max_idx), abs(H(1:max_idx)));
    xlabel('Frequency [Hz]');
    ylabel('Magnitude');
    title('New Signal Frequ Rep');
end


if 1==0
    fs = 40000;
    sample_code_len = 31;
    largest_test = 12*sample_code_len; % 372. To design with twice this is a code len of 744
    % Choose to design for a code length of 750. This is about 1.18M,
    
    lfsr_elements = 3:8;
    code_lens =  2.^lfsr_elements - 1;
    code_lens = 7:10:510;
    repeats = 1;
    samples_per_chip = 2;
    n = repeats * samples_per_chip .* code_lens;
%     n = linspace(1, 100, 100);
%     n = logspace(0,4, 10);  % mac makes this fast
%     n = 2^14
    ns = 1024;
    time_per_fft = ns/fs;
    %    brute_force Convolution in Time
    b_m = n.^2; % multiplications
    b_a = n.*(n-1); % additions
    b_total = b_m + b_a;

    % FFT 
    % Perform FFT
    f_m = (n/2).*log(n); % complex multiplications
    f_a = n.*log(n); % complex additions
%     f_total = f_m + f_a
    % Do FFT 2 times for f and g before we can multiply them together
    % Double the computations so we can inverse the FFT
    f_m_fft = 2 * f_m * 2;
    f_a_fft = 2 * f_m * 2;
    
    % Multiply each array against each other
    f_m_tot = f_m_fft + n;
    
    % convert to non-complex
    f_m_normal = 4 * f_m_tot;
    f_a_normal = 3 * f_m_tot + 2*f_a_fft;
    f_total = f_m_normal + f_a_normal;
    
    figure(1)
    plot(code_lens, f_total./b_total);
    xlabel('Code Lengths for Convolution (n)');
    ylabel('FFT Complexity / Naive Convolution');
    title('Complexity vs Number of Samples');
    
    
    figure(2)
    semilogy(code_lens, b_total*fs, code_lens, f_total*fs);
    xlabel('Code Lengths (n)');
    ylabel('Additions and Multiplications Per Second');
    title('Complexity vs Number of Samples');
    legend('Naive Method', 'FFT');
    %     f_total/b_total
    
    
    % DSP processors taken from http://www.ti.com/processors/dsp/getting-started/core-benchmarks.html
    %$600.00 per board http://www.ti.com/tool/TMDSEVM6678  $400.00 http://www.ti.com/tool/tmdsevm6657
    c66x_rate = 6.27 * 10^(-6); % seconds per thousand n   
    c66x = ones(1, length(n)) * round(1000/c66x_rate);
    
    %$195.00 http://www.ti.com/tool/tmdslcdk138 $195.00 http://www.ti.com/tool/tmdslcdk6748
    C674x_rate = 24.01 * 10^(-6);
    C674x = ones(1, length(n)) * round(1000/C674x_rate);
    
    % $335 http://www.ti.com/tool/k2gice
    Cortex_A15_rate = 43.92 * 10^(-6);
    Cortex_A15 = ones(1, length(n)) * round(1000/Cortex_A15_rate);
    
    %Atmel avr32718 at32UC3
    avr32718_rate = 2.9 * 10^(-3);
    avr32718 = ones(1, length(n)) * round(1000/avr32718_rate);
    
    % m4
    m4_cyc = 139898;
    m4_rate = 1.25 *10^(-6); % per second 
    m4_1_fft = m4_rate*m4_cyc;
    m4 = ones(1, length(n)) * round(1000/m4_1_fft);
    
    % m7
    m7_cyc =  87128;  % 17235; %  code len of 60 = 8050;  %
    m7_clock = 400*10^(6);
    m7_ips = (m7_clock/2)^(-1);
    m7_rate = m7_ips; % per second 
    m7_1_fft = m7_rate*m7_cyc;
    m7 = ones(1, length(n)) * round(1000/m7_1_fft);
    
    figure(3)
    semilogy(code_lens, n*fs, code_lens, c66x, code_lens, C674x, code_lens, Cortex_A15, code_lens, m4, code_lens, m7);
    xlabel('Code Lengths (n)');
    ylabel('FFT Points per Second');
    title('FFT Points per Second');
    legend('FFT Points', 'c66x', 'c674x', 'Cortex_A15', 'Cortex M4', 'Cortex M7')
end

if 1==1  % FFT percentage of operation time for various fft lengths
    n = [32, 64, 128, 256, 512, 1024];
    m5_cpf = [2577, 5282, 13823, 28000, 69253, 139898];
    m7_cpf = [1497, 3235, 8050, 17235, 41076, 87128];
    m5_clock = 48*10^6;  % per second
    m7_clock = 200*10^6;  % per second
    m5_time_per_fft = m5_cpf./m5_clock;
    m7_time_per_fft = m7_cpf./m7_clock;
    
    fs = 40000;
%     block_num = 6;
    block_time = n/fs;
%     m7_block_time = m7_time_per_fft(block_num);
%     m5_block_time = m5_time_per_fft(block_num);
    
%     m7_rat = m7_block_time/block_time
%     m5_rat = m5_block_time/block_time
    
    m7_rat = m7_time_per_fft./block_time;
    m5_rat = m5_time_per_fft./block_time;

    plot(n, m5_rat * 100, n, m7_rat * 100)
    xlabel('FFT Length');
    ylabel('Percent of Block Time')
end

if 1 ==0
    m5_cycles_per_1024 = 139898;
    m5_clock = 200*10^6;  % per second
    n = 1024;
    block_time = n/fs;
    code_len = 30:30:30*30;
%     adv_s_per_fft = n-code_len;
    
    
    
    dev_time_per_1024 = m5_cycles_per_1024*(n-code_len)/(m5_clock * 1000);
    
    m5_rat = dev_time_per_1024/block_time;
    
    plot(code_len, m5_rat)
    
    code_len = 30:30:30*30;
    adv_s_per_fft = n-code_len; 
   
    
    
end


% Theoretical accuracy single device
if 1==0
   chip_f = 1000:200:30000;
    speed = 340;
    % chip_f = 6 * 1000;
    bw = chip_f/2;
    chip_len = speed./chip_f;
    accuracy = (chip_len/10) * 100;
    figure(200)
    plot(chip_f, accuracy, '-rx')
    xlabel('Chipping Rate (Hz)');
    ylabel('Accuracy (cm)');
    title('Theoretical Single Device Accuracy per Chipping Rate') 
end



% fs = 80000;
% freq = 1000;
% tone = make_tone(freq, 0.1, fs);
% [H, F] = f_analysis(tone, fs);
% len_h = length(H);
% portion_to_disp = 0.8;
% max_idx = round(portion_to_disp*len_h);
% plot(F(1:max_idx), abs(H(1:max_idx)));
% xlabel('Frequency [Hz]');
% ylabel('Magnitude');
% title('New Signal Frequ Rep');
% figure(99)
% polarplot(ph_m(1:10) + 180, all_freq(1:10))


