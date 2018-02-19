function [amplitudes, phases] = read_sweep_points1(input_signal, output_signal, len_all_f, space_len, tone_len)
    % function [amplitudes, phases] = read_sweep_points(input_signal, output_signal, all_freq, space_len, tone_len, fs)
    % This function analyzes a frequency sweep and returns the amplitudes 
    % and phases for each point. It must have an input and output signal.
    % It finds the last point of the signal and works backwards to acquire
    % the whole signal. It chops up the signal into lengths of space_len +
    % tone_len. This identifies where tones start and end. It chops up the
    % signal into the given number of frequencies (len_all_f)
    % Written by Nephi Smith 15 Nov 2017
    
    % find last index of the input signal before it is padded with zeros
    ix_last = find(input_signal>max(input_signal)/2,1, 'last');
    
    % We will split the input and output signals into slices seg_length
    % long
    seg_length = space_len + tone_len;
    
    amplitudes = zeros(1, len_all_f);
    phases = zeros(1, len_all_f);
    % walk through and split and analyze the input and output signals
    for i = 1:len_all_f
       idx_start = ix_last - seg_length * i;
       if idx_start < 1
           idx_start = 1;
       end
       idx_end = idx_start + tone_len;
       
       tone_seg = input_signal(idx_start:idx_end);
       tone2 = output_signal(idx_start:idx_end);
       
       ft = fft(tone_seg);
       ft2 = fft(tone2);
       
       mid_idx = round(length(tone_seg)/2);
       [mag, idx] = max(abs(ft(1:mid_idx)));
       i_val = ft2(idx)/ft(idx);
       amplitudes(i) = abs(i_val);
       phases(i) = radtodeg(angle(i_val)); 
    end
end
       