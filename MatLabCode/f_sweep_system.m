function [amp_m_db, amp_std_db, ph_m, ph_std, all_freq] = f_sweep_system(start_f, end_f, f_points, dur_tone_s, fs, n, loudness, output_f)
    % function [amp_m_db, amp_std_db, ph_m, ph_std, all_freq] = f_sweep_system(start_f, end_f, f_points, dur_tone_s, fs, n, output_f)
    % This function performs a complete frequency sweep of the system and
    % returns the amplitude and phase response in mean and std in dB. It
    % takes a start frequency, end frequency, a number of frequency points
    % to be logrithmically spaced, a duration for each tone in seconds, a
    % sampling frequency (fs), a number of times to perform the sweep to
    % generate the standard deviations (n), and an output function such as
    % the MyDAQ to perform the analysis on.
    % Written by Nephi Smith 15 Nov 2017
    
    dur_start_s = 0.1;  % this is the delay before starting a sequence
    
    time_for_20m = 20/340;  % meters per speed of sound
    
    [points, all_freq, space_len, tone_len] = gen_sweep_points(time_for_20m, dur_start_s, dur_tone_s, start_f, end_f, f_points, fs);
    points = points * loudness;
    all_amp = zeros(n, f_points);
    all_ph = zeros(n, f_points);

    for i = 1:n    
        [tone, mod_tone] = output_f(fs, points);
        [amplitudes, phases] = read_sweep_points1(tone, mod_tone, f_points, space_len, tone_len);
        all_amp(i,:) = amplitudes;
        all_ph(i,:) = phases;
    end

    amplitudes_db = 20*log10(all_amp);

    amp_m_db = mean(amplitudes_db);
    amp_std_db = std(amplitudes_db);
    ph_m = mean(all_ph);
    ph_std = std(all_ph);
    
end    
    