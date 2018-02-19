function corr_data = corr_brute(ref_signal, captured_data)
    % function corr_data = corr_brute(ref_signal, captured_data)
    % This function performs cross-correlation with the ref_signal on the
    % captured_data. It returns the same number of points as captured_data
    % Written by Nephi Smith 18 Oct 2017
    
    corr_data = zeros(1, length(captured_data));
    % captured_data = [captured_data zeros(1, length_new_signal)];
    len_n = length(ref_signal);
    len_c = length(captured_data);
    for i = 1:(len_c - len_n - 1)
    %     vector_a = captured_data(i:i + len_n - 1);
    %     vector_b = new_signal;
    %     result_c = vector_a .* vector_b;
    %     sum_d = sum(result_c);
        corr_data(i) = sum( captured_data(i:i + len_n - 1) .* ref_signal);    
    end
end
