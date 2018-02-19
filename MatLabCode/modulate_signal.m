function [new_signal, new_fft] = modulate_signal(lfsr_code, samp_per_c, filter_len)
    % function [new_signal, new_fft] = modulate_signal(lfsr_code, samp_per_c, filter_len)
    % This function modulates a given LFSR code by upsampling, by padding
    % zeros between the frequency representation of the signal, and 
    % shifting signal location by padding zeros on the outside of the 
    % frequency representation of the signal
    % Written by Nephi Smith 18 Oct 2017
    
    % Normalize the code to [-1, 1]
    av_lfsr = lfsr_code - 0.5;
    lfsr_code = av_lfsr./abs(av_lfsr);
    len_code = length(lfsr_code);
    
    % Calculate Paramters 
    up_sample = samp_per_c;
    len_c = length(lfsr_code);
    
    % Perform FFT
    fft_c = fft(lfsr_code);
    
    % deal with odd fft lengths and Split Rh and Lh from each other
    idx_offset = rem(len_c, 2);
    mid_idx = idx_offset + floor(len_c/2);
    ns = up_sample*len_c;
    left_h = fft_c(1:mid_idx);
    right_h = fft_c(mid_idx + 1:len_c);
    
    % Padd zeros in the middle of the signal and around the sides
    num_mid_zeros = ns - len_c - filter_len*2;
    assert(num_mid_zeros >= 0, 'Sampling rate too low to modulate this frequency');
    mid_z = zeros(1, num_mid_zeros);
    z_filter = zeros(1, filter_len);
    n_fft_lfsr = [z_filter left_h mid_z right_h z_filter];

    % Scale the Signal
    new_signal = real(ifft(n_fft_lfsr)); % * up_sample;
    new_signal = new_signal/(max(new_signal));
    new_fft = n_fft_lfsr * up_sample;
end
