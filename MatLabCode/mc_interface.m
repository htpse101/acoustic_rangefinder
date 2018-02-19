

% This part reads in the output file of the microcontroller and operates on
% the data
fn_ctr_out = 'C:\STM32ToolChain\Downloads\ctr_out.csv';
fn_ctr_in = 'C:\STM32ToolChain\Downloads\ctr_in.csv';
fn_fs = 'C:\STM32ToolChain\Downloads\mc_fs.csv';



fs = csvread(fn_fs);

write_mode = 0;
bw = 7500; 
center_freq = 10005;
pn_code = [1,0,0,1,1,0,1,0,0,1,0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,1,1,1,1];
pn_code = [pn_code pn_code pn_code];
%     pn_code = [1,0,0,1,1,0];
adc_count = 2^12 - 1;
adc_mid = (adc_count-1)/2;
loudness = 1;%0.5;
signal = make_time_signal(bw, center_freq, pn_code, fs);  % make_tone(50000, 0.01, fs); %
signal = round(signal * adc_mid * loudness);
signal_tx = signal + adc_mid;
%     signal_tx = [zeros(1, round(length(signal)/30)) signal_tx];
data_len = length(signal_tx);
if 1 == 0
    
    
%     data_to_write = time_pad_signal(1, 0.5, fs, signal);
    data_to_write = signal_tx;  %[1,2,3,4,5,6,7,8];
    csvwrite(fn_ctr_in, [data_len data_to_write 0]);  % first number is array len
    figure(2);
    plot(signal_tx, 'x')
else  % must be read mode
    adc_input = csvread(fn_ctr_out);
    adc_input = adc_input - (max(adc_input) - min(adc_input))/2;
%     cor_data0b = corr_by_fft(signal, adc_input);
    cor = corr_brute(signal, adc_input);
    [max_val_ref, idx] = max(cor);
    sig_time = length(adc_input)/fs;
    time = 0:1/fs:(sig_time-1/fs);
    time_res = time(idx);
    dist = time_res * 340
    figure(1)
    subplot(2,1,1)
    plot(time, adc_input);
    subplot(2,1,2)
    plot(time, cor);
       
end
    




