function [new_signal] = make_time_signal(bw, center_freq, lfsr_code, fs)
    % function [new_signal] = make_time_signal(bw, center_freq, lfsr_code, fs)
    % This function returns a modulated time signal representation of the
    % lfsr_code. It needs a bandwidth (bw), a center frequency, an
    % lfsr_code, and a sampling frequency (fs).
    % Written by Nephi 16 Nov 2017
    % Do not modify these parameters
    chipping_f = bw*2;
    a_multi = (center_freq - 0.5*bw)/bw;  % internal multiplier. This centers everything

    filter_len_zeros = round(length(lfsr_code) * 0.5 * a_multi);  % Multiplying by 0.5 gives 1 signal width of zeros. Multiplying by a_multi positions the center where we want it
    samp_per_chip = round(fs/chipping_f);

    % Modulate The Signal
    [new_signal, new_fft] = modulate_signal(lfsr_code, samp_per_chip, filter_len_zeros);
end
