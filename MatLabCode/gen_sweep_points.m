function [points, all_freq, len_space, tone_len] = gen_sweep_points(dur_space_s, dur_start_s, dur_tone_s, start_f, end_f, f_points, fs)
    % function points = gen_sweep_points(dur_space_s, dur_start_s, dur_tone_s, start_f, end_f, f_points, fs)
    % This function creates an array for performing a frequency sweep of a
    % system. It spaces each frequency by a time (dur_space_s) and sets
    % each tone length (dur_tone_s). It delays the actual creation of the
    % tones by a time (dur_start_s). It creates a logrithmic scale from a
    % start frequency (start_f) to and ending frequency (end_f) and spaces
    % them with a given number of points (f_points). It also has a samping
    % frequeny (fs) to scale everything.
    % Written by Nephi Smith 15 Nov 2017
    
    % Create all frequency values
%     all_freq = logspace(log10(start_f), log10(end_f), f_points);
    all_freq = linspace(start_f, end_f, f_points);
    
    % determine number of points for each thing
    len_space = int32(round(dur_space_s*fs));
    tone_len = int32(round(dur_tone_s*fs));
    start_len = int32(round(dur_start_s*fs));
    pts_len = int32(round(start_len + len_space*f_points + tone_len*f_points));
    
    points = zeros(1, int32(pts_len));
    
    % stitch all of the frequencies together
    for i = 1:f_points
        freq = all_freq(i);
        tone = make_tone(freq, dur_tone_s, fs);
        start_idx = pts_len - (tone_len + len_space)*i;
        points(start_idx:start_idx + tone_len - 1) = tone;
    end
end
       