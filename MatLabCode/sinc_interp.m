function [new_signal, new_fft] = sinc_interp(code, total_size)
    % function [new_signal, new_fft] = sinc_interp(code, total_size)
    % This takes a code and interpolates it using sinc pulse interpolation.
    % The final number of samples is given by total_size
    % The total_size must be larger than the code.
    % This was only tested with an lsfr code that was odd. 
    % Written by Nephi Smith 10 Oct 2017
    %     lfsr_code = [1,0,0,1,1,0,1,0,0,1,0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,1,1,1,1]; % created with s=[1, 1, 0, 0, 1], t= [5, 2] using LFSR(s, t) 
    
%     %     total_size = 100;
%     lfsr_code = code;
%     fft_code = fft(lfsr_code);
%     %     figure(5)
%     %     plot(abs(fft_code));
%     len_code = length(fft_code);
%     new_fft = zeros(1, total_size);
%     mid_idx = round(len_code/2);
%     right_half = fft_code(1:mid_idx);
%     left_half = fft_code(mid_idx + 1:len_code);
%     new_fft(1:mid_idx) = right_half;  % apply right half
%     new_fft((total_size-length(right_half) + 2):total_size) = left_half;  % apply left half
%     %     figure(3)
%     %     plot(abs(new_fft));
% 
%     new_signal = ifft(new_fft);

    
    fft_code = fft(lfsr_code);
    len_code = length(fft_code);
    new_fft = zeros(1, total_size);
    mid_idx = round(len_code/2);
    right_half = fft_code(1:mid_idx);
    left_half = fft_code(mid_idx + 1:len_code);
    new_fft(1:mid_idx) = right_half;  % apply right half
    new_fft((total_size-length(right_half) + 2):total_size) = left_half;  % apply left half
    new_signal = ifft(new_fft);
    
    %     figure(1)
    %     plot(new_signal)

    %     figure(2)
    %     plot(lfsr_code)
end