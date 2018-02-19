
fs = 2^16;
dur_start_s = 0.2;  % this is the delay before starting a sequence
n = 5;   
loudness = 8;
time_for_20m = 30/340;  % meters per speed of sound

end_f = 22000;
f_points = 30;

lfsr_code = [1,0,0,1,1,0,1,0,0,1,0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,1,1,1,1]; % created with s=[1, 1, 0, 0, 1], t= [5, 2] using LFSR(s, t)% determine number of points for each thing
% lfsr_code = [lfsr_code lfsr_code];
% lfsr_code = [lfsr_code lfsr_code];
% lfsr_code = [lfsr_code lfsr_code lfsr_code];
code_len = length(lfsr_code);

bw_step = 1000;
bws = [1000:bw_step:7000];
len_bws = length(bws);

sig_str_m = zeros(len_bws, f_points);
sig_str_std = zeros(len_bws, f_points);
dist_m = zeros(len_bws, f_points);
dist_std = zeros(len_bws, f_points);
freqs = zeros(len_bws, f_points);

dur_space_s = time_for_20m;

betw_time = 1.6;
est_chip_len = fs/(2*1000);
est_time = betw_time*n*len_bws + (n * len_bws *(dur_start_s*fs + dur_space_s*fs*f_points + code_len*est_chip_len*f_points))/fs%  (time_for_20m*fs*f_points*code_len*)*len_bws;
for k = 1:length(bws)
    bw = bws(k);
    start_f = bw/2;
    all_center_freq = logspace(log10(start_f), log10(end_f), f_points);

    
    
    space_len = int32(dur_space_s*fs);
    chip_len = round(fs/(2*bw));
    tone_len = int32(code_len * chip_len);  % 100 because I counted wrong
    start_len = int32(dur_start_s*fs);

    pts_len = int32(start_len + space_len*f_points + tone_len*f_points);
    points = zeros(1, int32(pts_len));
    % stitch all of the frequencies together
    ref_tones = zeros(f_points, tone_len);
    for i = 1:f_points
        c_freq = all_center_freq(i);
        tone = make_time_signal(bw, c_freq, lfsr_code, fs);
        ref_tones(i, :) = tone;
        start_idx = pts_len - (tone_len + space_len)*i;
        points(start_idx:start_idx + tone_len - 1) = tone;
    end
    % end function

    % adjust volume;
    points = points * loudness;

    time_results = zeros(n, f_points);
    signal_strength = zeros(n, f_points);

    for j = 1:n    

        captured_data = [points;points*0.5];
        sig_time = length(captured_data)/fs;
        time = 0:1/fs:(sig_time-1/fs);
%         points = points;
%         [captured_data, time] = run_mydaq(fs, points);  %  s.startForeground();

        mod_points = captured_data(1,:);
        ref_points = captured_data(2,:);

    %     normalize each by their rms value;
        mod_points = mod_points/std(mod_points);
        ref_points = ref_points/std(ref_points);
        %     captured_data = run_mydaq(fs, points);
        % start function    [amplitudes, phases] = read_sweep_points1(tone, mod_tone, f_points, space_len, tone_len);

        ix_last = find(ref_points>max(abs(ref_points))/4,1, 'last') + round(space_len/2);

        % We will split the input and output signals into slices seg_length
        % long
        seg_length = space_len + tone_len;

        % walk through and split and analyze the input and output signals

        for i = 1:f_points
            idx_start = ix_last - seg_length * i;
            if idx_start < 1
                idx_start = 1;
            end
            idx_end = idx_start + seg_length;
            if idx_end > pts_len
                idx_end = int32(pts_len);
            end
            ref_tone = ref_tones(i, :);        
            ref_seg = ref_points(idx_start:idx_end);
            mod_seg = mod_points(idx_start:idx_end);

            % corr starts
            
            cor_data0b = corr_by_fft(ref_tone, mod_seg);
            cor_data1b_ref = corr_by_fft(ref_tone, ref_seg);
            
            % corr ends
            
            
            corr_data0 = corr_brute(ref_tone, mod_seg);
            corr_data1_ref = corr_brute(ref_tone, ref_seg);

            [max_val_ref, idx] = max(corr_data1_ref); 
            time_ref = time(idx);
            [max_val, idx] = max(corr_data0); 
            time_res = time(idx) - time_ref;
            time_results(j, i) = time_res;
            signal_strength(j, i) = max_val/max_val_ref;
        end
        % end function
    end
    
    freqs(k, :) = all_center_freq;
    
    times_m = mean(time_results);
    times_std = std(time_results);
    sig_str_m(k, :) = mean(signal_strength);
    sig_str_std(k, :) = std(signal_strength);

    % convert to distance measurement
    s_sound = 340;
    dist_m(k, :) = times_m * s_sound;
    dist_std(k, :) = times_std * s_sound;

    figure(k)
    subplot(1,2,1)
%     errorbar(all_center_freq, dist_m(k, :), dist_std(k, :));
    plot(all_center_freq, dist_std(k, :));
    xlabel('Frequency 10 kHz');
    ylabel('Distance (m) - Error below 0 m');
    title(['Consistency vs Center Frequency @BW ' num2str(bw/1000) ' kHz'])

    subplot(1,2,2)
    errorbar(all_center_freq, sig_str_m(k, :), sig_str_std(k, :));
    xlabel('Frequency 10 kHz');
    ylabel('Signal Strength Normalized to Reference');
    title(['Signal Strength vs Center Frequency @BW ' num2str(bw/1000) ' kHz'])

end
    
x = 2;


