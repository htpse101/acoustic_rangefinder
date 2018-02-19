function [amplitude, phase] = resp_amp_phase1(freq, dur_s, fs, output_f)
    % function [amplitude, phase] = response_amp_phase(freq, dur_s, fs, output_f)
    % This function runs this frequency through the 
    % Written by Nephi Smith 9 Nov 2017

    tone = make_tone(freq, dur_s, fs);
%     tone = [zeros(1, length(tone)), tone];
    % synthesize delayed tone
%     tone2 = output_f(fs, tone);
    [tone, tone2] = output_f(fs, tone);
    
    % find wanted frequency and extract the amplitude and phase
    mid_idx = round(length(tone)/2);
    ft = fft(tone);
    ft2 = fft(tone2);
    [mag, idx] = max(abs(ft(1:mid_idx)));
    i_val = ft2(idx)/ft(idx);
    amplitude = abs(i_val);
    phase = radtodeg(angle(i_val));    
end
