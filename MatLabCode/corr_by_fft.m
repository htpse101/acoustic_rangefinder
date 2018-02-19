function corr_res = corr_by_fft(ref_signal, captured_data)
    % function corr_fft = corr_fft(ref_signal, captured_data)
    % This function performs cross-correlation with the ref_signal on the
    % captured_data.
    % Written by Nephi Smith 1 Jan 2018
    
    ref_sig = [ fliplr(ref_signal) zeros(1, length(captured_data) - 1)];
    cap_sig = [ captured_data zeros(1, length(ref_signal) - 1)];
    
    ref_fft = fft(ref_sig);
    cap_fft = fft(cap_sig);
    
    corr_res = ifft(ref_fft.*cap_fft);
    
end
