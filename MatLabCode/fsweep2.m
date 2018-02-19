% function [amplitudes, phases, all_freq] = fsweep2(f_start, f_end, dur, n, fs, output_f)
    % function [amplitudes, phases, all_freq] = freq_sweep(start_f, end_f, step_f, dur_s, n, fs, output_f)
    % This performs a frequency sweep of the system and returns the
    % frequencies, amplitudes, and phases. Pass in each a start, step, and
    % end frequency. The duration for the tone (dur_s), the number of times
    % to take each measurement (n), the sampling frequency (fs), and the
    % output function (output_f). This supports using the mydaq or a custom
    % function that would filter the signal.
    % Currently, it is not optimized for speed and only returns one data
    % point for each experiment. It is possible to dramatically speed up
    % this function, but it would require normilizing the fft.
    % Written by Nephi Smith 9 Nov 2017

    fs = 45000;
    dur = 0.005; % smallest duration for 500 hz is 0.002 seconds

    f_start = 16000;
    f_end = 18000;
    len_tone = dur*fs;
    z_pad_len = 3;
    len_signal = len_tone * z_pad_len;  % this is the len of sent signal padded with zeros

    n = 3;
%     f_unit = fs/len_tone;
    f_unit = fs/len_signal;


    usable_fpts = z_pad_len;  % how many steps away from the center frequency we trust. 
    freq_step_len = 2*f_unit;

    all_freq = f_start:freq_step_len:f_end;
    len_allf = length(all_freq);
%     f_scale = 0:(fs/len_tone)/(z_pad_len + 1):fs/2;
%     f_scale = 0:(fs/len_tone)/z_pad_len:fs/2;
    f_scale = 0:fs/len_signal:fs/2;
    
    len_f_scale = length(f_scale);
    amp_resp = zeros(n, len_f_scale);
    ph_resp = zeros(n, len_f_scale);

    for i = 1:len_allf
        tone_f = all_freq(i);
        fre_upper = tone_f + (z_pad_len - 2) * f_unit;
        fre_center = tone_f;
        fre_lower = tone_f - (z_pad_len - 2) * f_unit;
        tone = make_tone(tone_f, dur, fs);
        tone = [zeros(1, len_tone*z_pad_len), tone];  % pad zeros to get more points in fft
        tone_out = filter_tone(fs, tone);
        
        % take fft
        fft_tone = fft(tone);
        fft_tone = fft_tone(1:len_signal/2 + 1);
        x = abs(fft_tone);
%         [a,b] = max(x(2:length(x)));
        [a,b] = max(x(2:length(x)));
%         b = b + 1;
        fft_tone_out = fft(tone_out);
        fft_tone_out = fft_tone_out(1:len_signal/2 + 1);
        figure(2)
        plot(f_scale, x);
        figure(1)
        % determine and record system_response
        if b > 2
            a_tri = abs(fft_tone_out(b-2:b+2)./fft_tone(b-2:b+2));
            amp_resp(1, b-2:b+2) = a_tri;

            a_ph_tri = radtodeg(angle(fft_tone_out(b-2:b+2)./fft_tone(b-2:b+2)));
            ph_resp(1, b-2:b+2) = a_ph_tri;
%             subplot(1,2,1)
%             plot(f_scale,amp_resp(1,:));
%             subplot(1,2,2)
%             plot(f_scale,ph_resp(1,:));
        end
        if b < 2
            apple = 2;
        end
%         plot(f_scale(b-5:b+5), x(b-5:b+5));
    end
subplot(1,2,1)
plot(f_scale,amp_resp(1,:));
subplot(1,2,2)
plot(f_scale,ph_resp(1,:));
    f = 1000; 

%     tone = make_tone(f, dur, fs);
%     len_t = length(tone);
%     tone = [zeros(1, len_t * 3) tone];
%     len_t = length(tone);
% 
%     fft_tone = fft(tone);
%     fft_tone = fft_tone(1:len_t/2 + 1);
% 
%     [a,b] = max(abs(fft_tone));
% 
%     f_scale = 0:fs/len_t:fs/2;
% 
% 
% 
%     bw = 2*fs/len_t;
%     figure(1);
%     subplot(1,2,1);
%     plot(f_scale, abs(fft_tone));
% 
% 
%     subplot(1,2,2);
%     [H, F] = f_analysis(tone, fs);
%     len_h = length(H);
%     portion_to_disp = 0.6;
%     max_idx = round(portion_to_disp*len_h);
%     plot(F(1:max_idx), abs(H(1:max_idx)));
%     xlabel('Frequency [Hz]');
%     ylabel('Magnitude');
%     title('New Signal Frequ Rep');
% 
% 
